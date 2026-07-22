#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "raspike_protocol_api.h"
#include "raspike_protocol_com.h"
#include "raspike_internal.h"

_Static_assert(RP_V2_FRAME_OVERHEAD + RP_V2_MAX_PAYLOAD == 256u,
               "RasPike v2 frame must fit in one 256-byte buffer");
_Static_assert(sizeof(RPProtocolSpikeStatus) <= RP_V2_MAX_PAYLOAD,
               "status payload exceeds protocol v2 maximum");
_Static_assert(sizeof(RPDriveTelemetry) <= RP_V2_MAX_PAYLOAD,
               "drive telemetry exceeds protocol v2 maximum");
_Static_assert(sizeof(RPRuntimeTelemetry) <= RP_V2_MAX_PAYLOAD,
               "runtime telemetry exceeds protocol v2 maximum");

#define LOGF(...) do { fprintf(stdout, __VA_ARGS__); fflush(stdout); } while (0)
#ifdef RASPIKE_TRACE
#define TRACEF(...) LOGF(__VA_ARGS__)
#else
#define TRACEF(...) do { } while (0)
#endif
#define RP_HANDSHAKE_TIMEOUT_MS 1500
#define RP_HANDSHAKE_ATTEMPTS 5
#define RP_HANDSHAKE_RETRY_DELAY_MS 100
#define RP_FRAME_TIMEOUT_MS 1000
#define RP_REQUEST_TIMEOUT_MS 1000

#ifndef SPIKE_EXPECTED_VERSION_MAJOR
#define SPIKE_EXPECTED_VERSION_MAJOR 0
#define SPIKE_EXPECTED_VERSION_MINOR 0
#define SPIKE_EXPECTED_VERSION_PATCH 0
#endif

typedef struct {
    uint64_t tx_frames;
    uint64_t rx_frames;
    uint64_t rx_legacy_frames;
    uint64_t crc_errors;
    uint64_t framing_errors;
    uint64_t session_errors;
    uint64_t request_timeouts;
    uint64_t request_retries;
    uint64_t stale_acks;
} RPInternalLinkStats;

static RPProtocolSpikeStatus *fgStatus;
static RPComDescriptor *fgDesc;
static RPRuntimeTelemetry fgRuntimeTelemetry;
static pthread_mutex_t fgRuntimeTelemetryMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t fgStatusMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t fgStatusCond = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t fgSendMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t fgSequenceMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t fgLinkStatsMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t fgLifecycleMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t fgLifecycleCond = PTHREAD_COND_INITIALIZER;

RPPortDevice gPortDevices[RP_PORT_NUMBER];

static _Atomic int fgHandshakeComplete;
static _Atomic int fgStopCommunication;
static int fgPortSynchronizationInitialized;
static _Atomic int fgUseProtocolV2;
static _Atomic uint32_t fgSessionId;
static _Atomic unsigned int fgCapabilities;
static int fgActiveReceivers;
static uint32_t fgNextSequence = 1;
static unsigned char fgVersionMajor;
static unsigned char fgVersionMinor;
static unsigned char fgVersionPatch;
static RPInternalLinkStats fgLinkStats;

static void stats_increment(uint64_t *counter)
{
    pthread_mutex_lock(&fgLinkStatsMutex);
    ++*counter;
    pthread_mutex_unlock(&fgLinkStatsMutex);
}

static struct timespec deadline_after_ms(clockid_t clock_id, int timeout_ms)
{
    struct timespec deadline;
    clock_gettime(clock_id, &deadline);
    deadline.tv_sec += timeout_ms / 1000;
    deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        ++deadline.tv_sec;
        deadline.tv_nsec -= 1000000000L;
    }
    return deadline;
}

int raspike_is_valid_port(RasPikePort port)
{
    return port < RP_DEVICE_PORT_COUNT || port == RP_PORT_NONE;
}

RPPortDevice *getDevice(RasPikePort port)
{
    if (port == RP_PORT_NONE) return &gPortDevices[RP_HUB_DEVICE_INDEX];
    if (port >= RP_DEVICE_PORT_COUNT) return NULL;
    return &gPortDevices[port];
}

static void init_port_device(RPPortDevice *dev)
{
    memset(dev, 0, sizeof(*dev));
    pthread_mutex_init(&dev->mutex, NULL);
    pthread_mutex_init(&dev->request_mutex, NULL);
    pthread_mutex_init(&dev->device_mutex, NULL);

    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
#ifdef CLOCK_MONOTONIC
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
#endif
    pthread_cond_init(&dev->cond, &attr);
    pthread_condattr_destroy(&attr);
}

static void reset_port_state(int error, int clear_devices)
{
    for (int i = 0; i < RP_PORT_NUMBER; ++i) {
        RPPortDevice *dev = &gPortDevices[i];
        pthread_mutex_lock(&dev->mutex);
        int canceled = dev->pending;
        if (canceled) {
            dev->ack_data = error;
            dev->ack_cmd = -1;
            dev->ack_sequence = dev->pending_sequence;
            dev->ack_session = fgSessionId;
            dev->pending = 0;
            pthread_cond_broadcast(&dev->cond);
        } else {
            dev->pending_sequence = 0;
            dev->ack_sequence = 0;
            dev->ack_session = 0;
            dev->ack_cmd = 0;
            dev->ack_data = 0;
        }
        pthread_mutex_unlock(&dev->mutex);
        if (clear_devices) {
            pthread_mutex_lock(&dev->device_mutex);
            memset(&dev->device, 0, sizeof(dev->device));
            pthread_mutex_unlock(&dev->device_mutex);
        }
    }
}

static uint32_t next_sequence(void)
{
    pthread_mutex_lock(&fgSequenceMutex);
    uint32_t sequence = fgNextSequence++;
    if (fgNextSequence == 0) fgNextSequence = 1;
    pthread_mutex_unlock(&fgSequenceMutex);
    return sequence;
}

static int send_legacy_frame(RasPikePort port, unsigned char cmd,
                             const unsigned char *payload, int size)
{
    if (!raspike_is_valid_port(port) || size < 0 || size > 255
        || (size > 0 && !payload)) return -EINVAL;
    unsigned char frame[4 + 255];
    frame[0] = RP_CMD_START;
    frame[1] = cmd;
    frame[2] = (unsigned char)size;
    frame[3] = port;
    if (size > 0) memcpy(frame + 4, payload, (size_t)size);
    return raspike_com_send(fgDesc, frame, size + 4);
}

static int send_v2_frame(RasPikePort port, unsigned char cmd,
                         const unsigned char *payload, int size,
                         uint32_t sequence, unsigned char flags)
{
    if (!raspike_is_valid_port(port) || size < 0
        || size > (int)RP_V2_MAX_PAYLOAD || (size > 0 && !payload)) {
        return -EINVAL;
    }

    unsigned char frame[RP_V2_FRAME_OVERHEAD + RP_V2_MAX_PAYLOAD];
    frame[0] = RP_V2_START;
    frame[RP_V2_OFFSET_VERSION] = RP_V2_VERSION;
    frame[RP_V2_OFFSET_FLAGS] = flags;
    frame[RP_V2_OFFSET_CMD] = cmd;
    frame[RP_V2_OFFSET_PORT] = port;
    frame[RP_V2_OFFSET_SIZE] = (unsigned char)size;
    rp_v2_put_u32(frame + RP_V2_OFFSET_SEQUENCE, sequence);
    rp_v2_put_u32(frame + RP_V2_OFFSET_SESSION, fgSessionId);
    if (size > 0) memcpy(frame + RP_V2_OFFSET_PAYLOAD, payload, (size_t)size);

    size_t crc_input_size = (RP_V2_HEADER_SIZE - 1u) + (size_t)size;
    uint16_t crc = rp_v2_crc16(frame + 1, crc_input_size);
    size_t crc_offset = RP_V2_OFFSET_PAYLOAD + (size_t)size;
    rp_v2_put_u16(frame + crc_offset, crc);
    int sent = raspike_com_send(fgDesc, frame, (int)(crc_offset + RP_V2_CRC_SIZE));
    if (cmd == RP_CMD_ID_MOT_CFG || cmd == RP_CMD_ID_ACK) {
        TRACEF("RASPIKE_TX_V2,cmd=%u,port=%u,sequence=%lu,session=%lu,size=%d,sent=%d\n",
             cmd, port, (unsigned long)sequence, (unsigned long)fgSessionId,
             size, sent);
    }
    return sent;
}

static int send_frame(RasPikePort port, unsigned char cmd,
                      const unsigned char *payload, int size,
                      uint32_t sequence, unsigned char flags,
                      int flush_now)
{
    pthread_mutex_lock(&fgSendMutex);
    if (!fgDesc || atomic_load(&fgStopCommunication)) {
        pthread_mutex_unlock(&fgSendMutex);
        return -ENOTCONN;
    }
    int ret = atomic_load(&fgUseProtocolV2)
        ? send_v2_frame(port, cmd, payload, size, sequence, flags)
        : send_legacy_frame(port, cmd, payload, size);
    if (ret >= 0 && flush_now) {
        int flush_result = raspike_com_flush(fgDesc);
        if (flush_result < 0) ret = flush_result;
    }
    pthread_mutex_unlock(&fgSendMutex);
    if (ret >= 0) stats_increment(&fgLinkStats.tx_frames);
    return ret;
}

static int scan_for_byte(unsigned char target, int timeout_ms)
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);
    int elapsed_ms = 0;
    while (elapsed_ms < timeout_ms) {
        unsigned char byte = 0;
        int ret = raspike_com_receive_timeout(fgDesc, &byte, 1, 100);
        if (ret == 1 && byte == target) return 0;
        if (ret < 0 && ret != -ETIMEDOUT && ret != -EAGAIN) return ret;
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        elapsed_ms = (int)((now.tv_sec - start.tv_sec) * 1000
                   + (now.tv_nsec - start.tv_nsec) / 1000000);
    }
    return -ETIMEDOUT;
}

static int perform_handshake_once(void)
{
    const unsigned char init[2] = {RP_CMD_INIT, RP_CMD_INIT_MAGIC};
    int sent = raspike_com_send(fgDesc, init, sizeof(init));
    if (sent < 0) return sent;
    int flush_result = raspike_com_flush(fgDesc);
    if (flush_result < 0) return flush_result;

    int found = scan_for_byte(RP_CMD_INIT, RP_HANDSHAKE_TIMEOUT_MS);
    if (found < 0) return found;

    unsigned char magic = 0;
    int ret = raspike_com_receive_timeout(fgDesc, &magic, 1, 500);
    if (ret < 0) return ret;
    if (magic != RP_CMD_INIT_MAGIC) return -EPROTO;

    unsigned char version[3];
    ret = raspike_com_receive_timeout(fgDesc, version, sizeof(version), 500);
    if (ret < 0) return ret;
    fgVersionMajor = version[0];
    fgVersionMinor = version[1];
    fgVersionPatch = version[2];
    return 0;
}

static int perform_handshake(void)
{
    int last_error = -ETIMEDOUT;
    for (int attempt = 1; attempt <= RP_HANDSHAKE_ATTEMPTS; ++attempt) {
        int discard_result = raspike_com_discard_input(fgDesc);
        if (discard_result < 0 && discard_result != -ENOTTY) {
            last_error = discard_result;
        }
        int result = perform_handshake_once();
        if (result == 0) {
            if (attempt > 1) {
                LOGF("RASPIKE_HANDSHAKE_RECOVERED,attempt=%d\n", attempt);
            }
            return 0;
        }
        last_error = result;
        LOGF("RASPIKE_HANDSHAKE_RETRY,attempt=%d,error=%d\n", attempt, -result);
        if (attempt < RP_HANDSHAKE_ATTEMPTS) {
            struct timespec delay = {
                .tv_sec = 0,
                .tv_nsec = RP_HANDSHAKE_RETRY_DELAY_MS * 1000000L,
            };
            nanosleep(&delay, NULL);
        }
    }
    return last_error;
}

static int receive_legacy_hello(void)
{
    int ret = scan_for_byte(RP_CMD_START, 2000);
    if (ret < 0) return ret;

    unsigned char header[3];
    ret = raspike_com_receive_timeout(fgDesc, header, sizeof(header), 1000);
    if (ret < 0) return ret;
    unsigned char cmd = header[0];
    unsigned char size = header[1];
    RasPikePort port = header[2];
    if (cmd != RP_CMD_ID_LINK_HELLO || port != RP_PORT_NONE
        || size != RP_V2_HELLO_PAYLOAD_SIZE) {
        return -EPROTO;
    }

    unsigned char payload[RP_V2_HELLO_PAYLOAD_SIZE];
    ret = raspike_com_receive_timeout(fgDesc, payload, sizeof(payload), 1000);
    if (ret < 0) return ret;
    if (payload[0] != RP_V2_VERSION) return -EPROTONOSUPPORT;

    fgCapabilities = payload[1];
    uint16_t max_payload = rp_v2_get_u16(payload + 2);
    fgSessionId = rp_v2_get_u32(payload + 4);
    uint32_t status_period_us = rp_v2_get_u32(payload + 8);
    if (!(fgCapabilities & RP_V2_CAP_FRAMING) || fgSessionId == 0
        || max_payload < sizeof(RPProtocolSpikeStatus)) {
        return -EPROTONOSUPPORT;
    }
    fgUseProtocolV2 = 1;
    LOGF("RASPIKE_LINK_READY,protocol=2,session=%lu,capabilities=0x%02x,max_payload=%u,status_period_us=%lu\n",
         (unsigned long)fgSessionId, fgCapabilities, max_payload,
         (unsigned long)status_period_us);
    return 0;
}

int raspike_prot_init(RPComDescriptor *desc)
{
    if (!desc) return -EINVAL;
    pthread_mutex_lock(&fgLifecycleMutex);
    if (fgDesc) {
        pthread_mutex_unlock(&fgLifecycleMutex);
        return -EBUSY;
    }
    fgDesc = desc;
    atomic_store(&fgStopCommunication, 0);
    atomic_store(&fgHandshakeComplete, 0);
    atomic_store(&fgUseProtocolV2, 0);
    atomic_store(&fgSessionId, 0);
    atomic_store(&fgCapabilities, 0);
    fgActiveReceivers = 0;
    pthread_mutex_unlock(&fgLifecycleMutex);
    fgNextSequence = 1;
    memset(&fgLinkStats, 0, sizeof(fgLinkStats));
    memset(&fgRuntimeTelemetry, 0, sizeof(fgRuntimeTelemetry));

    fgStatus = calloc(1, sizeof(*fgStatus));
    if (!fgStatus) {
        pthread_mutex_lock(&fgLifecycleMutex);
        fgDesc = NULL;
        pthread_mutex_unlock(&fgLifecycleMutex);
        return -ENOMEM;
    }
    if (!fgPortSynchronizationInitialized) {
        for (int i = 0; i < RP_PORT_NUMBER; ++i) init_port_device(&gPortDevices[i]);
        fgPortSynchronizationInitialized = 1;
    } else {
        reset_port_state(-ENOTCONN, 1);
    }

    int ret = perform_handshake();
    if (ret < 0) {
        LOGF("RASPIKE_HANDSHAKE_FAILED,error=%d\n", -ret);
        free(fgStatus);
        fgStatus = NULL;
        pthread_mutex_lock(&fgLifecycleMutex);
        fgDesc = NULL;
        pthread_mutex_unlock(&fgLifecycleMutex);
        return ret;
    }

    LOGF("SPIKE asp.bin version=%u.%u.%u expected=%u.%u.%u\n",
         fgVersionMajor, fgVersionMinor, fgVersionPatch,
         SPIKE_EXPECTED_VERSION_MAJOR, SPIKE_EXPECTED_VERSION_MINOR,
         SPIKE_EXPECTED_VERSION_PATCH);
    if (fgVersionMajor != SPIKE_EXPECTED_VERSION_MAJOR
        || fgVersionMinor != SPIKE_EXPECTED_VERSION_MINOR
        || fgVersionPatch != SPIKE_EXPECTED_VERSION_PATCH) {
        LOGF("SPIKE version mismatched! update asp.bin\n");
        free(fgStatus);
        fgStatus = NULL;
        pthread_mutex_lock(&fgLifecycleMutex);
        fgDesc = NULL;
        pthread_mutex_unlock(&fgLifecycleMutex);
        return -EPROTONOSUPPORT;
    }

    ret = receive_legacy_hello();
    if (ret < 0) {
        LOGF("RASPIKE_V2_NEGOTIATION_FAILED,error=%d\n", -ret);
        free(fgStatus);
        fgStatus = NULL;
        pthread_mutex_lock(&fgLifecycleMutex);
        fgDesc = NULL;
        pthread_mutex_unlock(&fgLifecycleMutex);
        return ret;
    }

    fgHandshakeComplete = 1;
    raspike_usb_set_mode(RASPIKE_USB_MODE_NORMAL);
    return 0;
}

int raspike_prot_shutdown(void)
{
    atomic_store(&fgStopCommunication, 1);

    pthread_mutex_lock(&fgLifecycleMutex);
    while (fgActiveReceivers > 0) {
        pthread_cond_wait(&fgLifecycleCond, &fgLifecycleMutex);
    }
    pthread_mutex_lock(&fgSendMutex);
    RPComDescriptor *desc = fgDesc;
    fgDesc = NULL;
    pthread_mutex_unlock(&fgLifecycleMutex);

    reset_port_state(-ENOTCONN, 1);
    if (desc) {
        (void)raspike_com_flush(desc);
        (void)raspike_com_close(desc);
    }
    pthread_mutex_unlock(&fgSendMutex);

    pthread_mutex_lock(&fgStatusMutex);
    free(fgStatus);
    fgStatus = NULL;
    pthread_mutex_unlock(&fgStatusMutex);
    atomic_store(&fgHandshakeComplete, 0);
    atomic_store(&fgUseProtocolV2, 0);
    atomic_store(&fgSessionId, 0);
    return 0;
}

RPProtocolSpikeStatus *raspike_prot_get_saved_status(void)
{
    return fgStatus;
}

int raspike_prot_lock_status(void)
{
    return pthread_mutex_lock(&fgStatusMutex);
}

int raspike_prot_unlock_status(void)
{
    return pthread_mutex_unlock(&fgStatusMutex);
}

int raspike_prot_send(RasPikePort port, unsigned char cmdid,
                      const unsigned char *buf, int size)
{
    return send_frame(port, cmdid, buf, size, next_sequence(), 0, 0);
}

int raspike_prot_send_priority(RasPikePort port, unsigned char cmdid,
                               const unsigned char *buf, int size)
{
    return send_frame(port, cmdid, buf, size, next_sequence(),
                      RP_V2_FLAG_HIGH_PRIORITY, 1);
}

int raspike_request(RasPikePort port, unsigned char cmd,
                    const unsigned char *data, int size, int timeout_ms)
{
    if (!raspike_is_valid_port(port)) return -EINVAL;
    RPPortDevice *dev = getDevice(port);
    if (!dev) return -EINVAL;
    if (timeout_ms <= 0) timeout_ms = RP_REQUEST_TIMEOUT_MS;

    pthread_mutex_lock(&dev->request_mutex);
    uint32_t sequence = next_sequence();
    pthread_mutex_lock(&dev->mutex);
    dev->pending = 1;
    dev->pending_sequence = sequence;
    dev->ack_sequence = 0;
    dev->ack_session = 0;
    dev->ack_cmd = 0;
    dev->ack_data = 0;
    pthread_mutex_unlock(&dev->mutex);

    const int max_attempts = fgUseProtocolV2 ? 2 : 1;
    int result = -ETIMEDOUT;
    int completed = 0;

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        pthread_mutex_lock(&dev->mutex);
        int still_pending = dev->pending;
        pthread_mutex_unlock(&dev->mutex);
        if (!still_pending) {
            completed = 1;
            break;
        }

        int sent = send_frame(port, cmd, data, size, sequence,
                              RP_V2_FLAG_HIGH_PRIORITY, 1);
        if (sent < 0) {
            result = sent;
            break;
        }

        struct timespec deadline = deadline_after_ms(CLOCK_MONOTONIC, timeout_ms);
        pthread_mutex_lock(&dev->mutex);
        while (dev->pending) {
            int wait_result = pthread_cond_timedwait(
                &dev->cond, &dev->mutex, &deadline);
            if (wait_result == ETIMEDOUT) break;
            if (wait_result != 0 && wait_result != EINTR) {
                result = -wait_result;
                break;
            }
        }
        if (!dev->pending) {
            completed = 1;
            pthread_mutex_unlock(&dev->mutex);
            break;
        }
        pthread_mutex_unlock(&dev->mutex);

        if (attempt + 1 < max_attempts) {
            stats_increment(&fgLinkStats.request_retries);
            LOGF("RASPIKE_REQUEST_RETRY,port=%u,cmd=%u,sequence=%lu,attempt=%d,session=%lu\n",
                 port, cmd, (unsigned long)sequence, attempt + 2, (unsigned long)fgSessionId);
        }
    }


    pthread_mutex_lock(&dev->mutex);
    if (completed && dev->ack_cmd == -1) {
        result = dev->ack_data;
    } else if (completed
        && dev->ack_cmd == cmd
        && (!atomic_load(&fgUseProtocolV2)
            || (dev->ack_sequence == sequence
                && dev->ack_session == atomic_load(&fgSessionId)))) {
        result = dev->ack_data;
    } else if (result == -ETIMEDOUT) {
        stats_increment(&fgLinkStats.request_timeouts);
        LOGF("RASPIKE_REQUEST_TIMEOUT,port=%u,cmd=%u,sequence=%lu,session=%lu\n",
             port, cmd, (unsigned long)sequence, (unsigned long)fgSessionId);
    }
    dev->pending = 0;
    pthread_mutex_unlock(&dev->mutex);
    pthread_mutex_unlock(&dev->request_mutex);
    TRACEF("RASPIKE_REQUEST_RETURN,port=%u,cmd=%u,sequence=%lu,result=%d,completed=%d\n",
         port, cmd, (unsigned long)sequence, result, completed);
    return result;
}


static void process_hello_payload(const unsigned char *buf, int size)
{
    if (size != RP_V2_HELLO_PAYLOAD_SIZE || buf[0] != RP_V2_VERSION) return;
    uint32_t session = rp_v2_get_u32(buf + 4);
    if (session == 0 || session == fgSessionId) return;

    uint32_t old_session = fgSessionId;
    fgSessionId = session;
    fgCapabilities = buf[1];
    reset_port_state(-ESTALE, 0);
    if (fgStatus) {
        pthread_mutex_lock(&fgStatusMutex);
        memset(fgStatus, 0, sizeof(*fgStatus));
        pthread_cond_broadcast(&fgStatusCond);
        pthread_mutex_unlock(&fgStatusMutex);
    }
    pthread_mutex_lock(&fgRuntimeTelemetryMutex);
    memset(&fgRuntimeTelemetry, 0, sizeof(fgRuntimeTelemetry));
    pthread_mutex_unlock(&fgRuntimeTelemetryMutex);
    LOGF("RASPIKE_SESSION_CHANGED,old=%lu,new=%lu\n",
         (unsigned long)old_session, (unsigned long)session);
}

static int process_command(RasPikePort port, unsigned char cmd,
                           unsigned char flags, uint32_t sequence,
                           uint32_t session, const unsigned char *buf, int size)
{
    if (cmd == RP_CMD_ID_LINK_HELLO) {
        process_hello_payload(buf, size);
        return 0;
    }
    if (!raspike_is_valid_port(port)) {
        stats_increment(&fgLinkStats.framing_errors);
        return -EINVAL;
    }
    if (fgUseProtocolV2 && session != fgSessionId) {
        stats_increment(&fgLinkStats.session_errors);
        return -ESTALE;
    }

    switch (cmd) {
    case RP_CMD_ID_ALL_STATUS:
        if (size != (int)sizeof(*fgStatus)) return -EMSGSIZE;
        pthread_mutex_lock(&fgStatusMutex);
        memcpy(fgStatus, buf, sizeof(*fgStatus));
        pthread_cond_broadcast(&fgStatusCond);
        pthread_mutex_unlock(&fgStatusMutex);
        return 0;

    case RP_CMD_ID_RUNTIME_TELEMETRY:
        if (size != (int)sizeof(fgRuntimeTelemetry)) return -EMSGSIZE;
        pthread_mutex_lock(&fgRuntimeTelemetryMutex);
        memcpy(&fgRuntimeTelemetry, buf, sizeof(fgRuntimeTelemetry));
        pthread_mutex_unlock(&fgRuntimeTelemetryMutex);
        return 0;

    case RP_CMD_ID_DRIVE_TELEMETRY: {
        if (size != (int)sizeof(RPDriveTelemetry)) return -EMSGSIZE;
        RPDriveTelemetry telemetry;
        memcpy(&telemetry, buf, sizeof(telemetry));
        pthread_mutex_lock(&fgStatusMutex);
        for (unsigned int slot = 0; slot < RP_DRIVE_TELEMETRY_MOTOR_SLOTS; ++slot) {
            if (!(telemetry.flags & (1u << slot))) continue;
            RasPikePort motor_port = telemetry.motor_port[slot];
            if (motor_port >= RP_DEVICE_PORT_COUNT) continue;
            RPProtocolPortStatus *status = &fgStatus->ports[motor_port];
            status->port = motor_port;
            memcpy(status->data + RP_MOTOR_INDEX_COUNT,
                   &telemetry.motor_count_deg[slot], sizeof(int32_t));
            memcpy(status->data + RP_MOTOR_INDEX_SPEED,
                   &telemetry.motor_speed_deg_s[slot], sizeof(int32_t));
            memcpy(status->data + RP_MOTOR_INDEX_POWER,
                   &telemetry.motor_power[slot], sizeof(int16_t));
        }
        if (telemetry.flags & RP_DRIVE_TELEMETRY_COLOR_VALID) {
            RasPikePort color_port = telemetry.color_port;
            if (color_port < RP_DEVICE_PORT_COUNT) {
                RPProtocolPortStatus *status = &fgStatus->ports[color_port];
                int32_t reflection = telemetry.reflection;
                status->port = color_port;
                memcpy(status->data, &reflection, sizeof(reflection));
            }
        }
        pthread_cond_broadcast(&fgStatusCond);
        pthread_mutex_unlock(&fgStatusMutex);
        return 0;
    }

    case RP_CMD_ID_ACK: {
        TRACEF("RASPIKE_RX_ACK,port=%u,sequence=%lu,session=%lu,size=%d\n",
             port, (unsigned long)sequence, (unsigned long)session, size);
        if (size != 8) return -EMSGSIZE;
        RPPortDevice *dev = getDevice(port);
        if (!dev) return -EINVAL;
        int32_t ack_cmd;
        int32_t ack_data;
        memcpy(&ack_cmd, buf, sizeof(ack_cmd));
        memcpy(&ack_data, buf + 4, sizeof(ack_data));

        pthread_mutex_lock(&dev->mutex);
        int matches = dev->pending && ack_cmd >= 0
            && (!atomic_load(&fgUseProtocolV2)
                || (sequence == dev->pending_sequence
                    && session == atomic_load(&fgSessionId)));
        TRACEF("RASPIKE_ACK_MATCH,port=%u,ack_cmd=%ld,pending=%d,pending_sequence=%lu,rx_sequence=%lu,match=%d\n",
             port, (long)ack_cmd, dev->pending,
             (unsigned long)dev->pending_sequence,
             (unsigned long)sequence, matches);
        if (matches) {
            dev->ack_cmd = ack_cmd;
            dev->ack_data = ack_data;
            dev->ack_sequence = sequence;
            dev->ack_session = session;
            dev->pending = 0;
            pthread_cond_broadcast(&dev->cond);
        } else {
            stats_increment(&fgLinkStats.stale_acks);
        }
        pthread_mutex_unlock(&dev->mutex);
        return matches ? 0 : -ESTALE;
    }

    case RP_CMD_ID_LINK_STATS:
        return 0;

    default:
        (void)flags;
        return -ENOMSG;
    }
}

static int receive_legacy_frame(void)
{
    unsigned char header[3];
    int ret = raspike_com_receive_timeout(fgDesc, header, sizeof(header), RP_FRAME_TIMEOUT_MS);
    if (ret < 0) return ret;
    unsigned char cmd = header[0];
    int size = header[1];
    RasPikePort port = header[2];
    if (size > RP_PROTOCOL_BUFMAX || !raspike_is_valid_port(port)) {
        stats_increment(&fgLinkStats.framing_errors);
        return -EPROTO;
    }
    unsigned char payload[RP_PROTOCOL_BUFMAX];
    if (size > 0) {
        ret = raspike_com_receive_timeout(fgDesc, payload, size, RP_FRAME_TIMEOUT_MS);
        if (ret < 0) return ret;
    }
    stats_increment(&fgLinkStats.rx_legacy_frames);
    return process_command(port, cmd, 0, 0, fgSessionId, payload, size);
}

static int discard_receive_bytes(int size)
{
    unsigned char scratch[64];
    while (size > 0) {
        int chunk = size < (int)sizeof(scratch) ? size : (int)sizeof(scratch);
        int result = raspike_com_receive_timeout(fgDesc, scratch, chunk,
                                                 RP_FRAME_TIMEOUT_MS);
        if (result < 0) return result;
        size -= result;
    }
    return 0;
}

static int receive_v2_frame(void)
{
    unsigned char frame[RP_V2_FRAME_OVERHEAD + RP_V2_MAX_PAYLOAD];
    frame[0] = RP_V2_START;
    int ret = raspike_com_receive_timeout(fgDesc, frame + 1,
                                          RP_V2_HEADER_SIZE - 1,
                                          RP_FRAME_TIMEOUT_MS);
    if (ret < 0) return ret;
    int size = frame[RP_V2_OFFSET_SIZE];
    if (frame[RP_V2_OFFSET_VERSION] != RP_V2_VERSION) {
        (void)discard_receive_bytes(size + RP_V2_CRC_SIZE);
        stats_increment(&fgLinkStats.framing_errors);
        return -EPROTONOSUPPORT;
    }

    if (size > (int)RP_V2_MAX_PAYLOAD) {
        (void)discard_receive_bytes(size + RP_V2_CRC_SIZE);
        stats_increment(&fgLinkStats.framing_errors);
        return -EMSGSIZE;
    }
    ret = raspike_com_receive_timeout(fgDesc, frame + RP_V2_OFFSET_PAYLOAD,
                                      size + RP_V2_CRC_SIZE,
                                      RP_FRAME_TIMEOUT_MS);
    if (ret < 0) return ret;

    size_t crc_offset = RP_V2_OFFSET_PAYLOAD + (size_t)size;
    uint16_t received_crc = rp_v2_get_u16(frame + crc_offset);
    uint16_t calculated_crc = rp_v2_crc16(frame + 1,
        (RP_V2_HEADER_SIZE - 1u) + (size_t)size);
    if (received_crc != calculated_crc) {
        stats_increment(&fgLinkStats.crc_errors);
        return -EBADMSG;
    }

    unsigned char flags = frame[RP_V2_OFFSET_FLAGS];
    unsigned char cmd = frame[RP_V2_OFFSET_CMD];
    RasPikePort port = frame[RP_V2_OFFSET_PORT];
    uint32_t sequence = rp_v2_get_u32(frame + RP_V2_OFFSET_SEQUENCE);
    uint32_t session = rp_v2_get_u32(frame + RP_V2_OFFSET_SESSION);
    return process_command(port, cmd, flags, sequence, session,
                           frame + RP_V2_OFFSET_PAYLOAD, size);
}

int raspike_prot_receive(void)
{
    pthread_mutex_lock(&fgLifecycleMutex);
    if (atomic_load(&fgStopCommunication) || !fgDesc
        || !atomic_load(&fgHandshakeComplete)) {
        pthread_mutex_unlock(&fgLifecycleMutex);
        return -ENOTCONN;
    }
    ++fgActiveReceivers;
    pthread_mutex_unlock(&fgLifecycleMutex);

    int result = 0;
    unsigned char start = 0;
    int ret = raspike_com_receive_timeout(fgDesc, &start, 1, 500);
    if (ret == -ETIMEDOUT || ret == -EAGAIN) {
        result = 0;
        goto done;
    }
    if (ret < 0) {
        result = ret;
        goto done;
    }

    if (start == RP_V2_START) {
        result = receive_v2_frame();
    } else if (start == RP_CMD_START) {
        result = receive_legacy_frame();
    } else {
        stats_increment(&fgLinkStats.framing_errors);
        result = -EAGAIN;
    }

    if (result == 0 || result == -ENOMSG || result == -ESTALE) {
        stats_increment(&fgLinkStats.rx_frames);
    }

done:
    pthread_mutex_lock(&fgLifecycleMutex);
    --fgActiveReceivers;
    if (fgActiveReceivers == 0) pthread_cond_broadcast(&fgLifecycleCond);
    pthread_mutex_unlock(&fgLifecycleMutex);
    return result;
}

int raspike_wait_port_cmd_change(RasPikePort port, unsigned char wait_cmd)
{
    if (port >= RP_DEVICE_PORT_COUNT || !fgStatus) return -EINVAL;
    struct timespec deadline = deadline_after_ms(CLOCK_REALTIME, 1000);
    pthread_mutex_lock(&fgStatusMutex);
    while (fgStatus->ports[port].cmd != wait_cmd) {
        int ret = pthread_cond_timedwait(&fgStatusCond, &fgStatusMutex, &deadline);
        if (ret == ETIMEDOUT) {
            pthread_mutex_unlock(&fgStatusMutex);
            return -ETIMEDOUT;
        }
        if (ret != 0 && ret != EINTR) {
            pthread_mutex_unlock(&fgStatusMutex);
            return -ret;
        }
    }
    pthread_mutex_unlock(&fgStatusMutex);
    return 0;
}

int raspike_port_com_change_if_needed(RasPikePort port, unsigned char wait_cmd)
{
    if (port >= RP_DEVICE_PORT_COUNT || !fgStatus) return -EINVAL;
    pthread_mutex_lock(&fgStatusMutex);
    int already_set = fgStatus->ports[port].cmd == wait_cmd;
    pthread_mutex_unlock(&fgStatusMutex);
    if (already_set) return 0;

    int sent = raspike_prot_send_priority(port, wait_cmd, NULL, 0);
    if (sent < 0) return sent;
    return raspike_wait_port_cmd_change(port, wait_cmd);
}

int raspike_drive_configure(void)
{
    return raspike_request(RP_PORT_NONE, RP_CMD_ID_DRIVE_CONFIG, NULL, 0, 3000);
}

int raspike_drive_send_command(const RPRealtimeDriveCommand *command)
{
    if (!command) return -EINVAL;
    return raspike_prot_send_priority(RP_PORT_NONE, RP_CMD_ID_DRIVE_COMMAND,
                                      (const unsigned char *)command,
                                      sizeof(*command));
}


int raspike_runtime_inject_fault(uint32_t fault_mask,
                                 uint32_t duration_cycles)
{
    const RPRuntimeFaultCommand command = {
        .fault_mask = fault_mask,
        .duration_cycles = duration_cycles,
    };
    return raspike_request(RP_PORT_NONE, RP_CMD_ID_RUNTIME_FAULT,
                           (const unsigned char *)&command,
                           sizeof(command), 1000);
}

int raspike_runtime_get_telemetry(RPRuntimeTelemetry *telemetry)
{
    if (!telemetry) return EINVAL;
    pthread_mutex_lock(&fgRuntimeTelemetryMutex);
    *telemetry = fgRuntimeTelemetry;
    pthread_mutex_unlock(&fgRuntimeTelemetryMutex);
    return telemetry->sequence ? 0 : EAGAIN;
}

int raspike_link_get_stats(RasPikeLinkStats *stats)
{
    if (!stats) return EINVAL;
    pthread_mutex_lock(&fgLinkStatsMutex);
    stats->session_id = fgSessionId;
    stats->protocol_version = fgUseProtocolV2 ? RP_V2_VERSION : 1;
    stats->capabilities = fgCapabilities;
    stats->tx_frames = fgLinkStats.tx_frames;
    stats->rx_frames = fgLinkStats.rx_frames;
    stats->crc_errors = fgLinkStats.crc_errors;
    stats->framing_errors = fgLinkStats.framing_errors;
    stats->session_errors = fgLinkStats.session_errors;
    stats->request_timeouts = fgLinkStats.request_timeouts;
    stats->request_retries = fgLinkStats.request_retries;
    stats->stale_acks = fgLinkStats.stale_acks;
    pthread_mutex_unlock(&fgLinkStatsMutex);
    return 0;
}
