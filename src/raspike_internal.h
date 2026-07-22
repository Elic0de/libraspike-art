#ifndef __RASPIKE_INTERNAL_H_
#define __RASPIKE_INTERNAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "raspike_protocol_com.h"

#define DECLARE_DEVICE_TYPE_IN_FILE(device_type) \
    static unsigned char fgDeviceType = (device_type);

struct _pup_device_t {
    RasPikePort port_id;
    unsigned char device_type;
    unsigned char cmd;
    int32_t power;
};

typedef struct {
    struct _pup_device_t device;
    pthread_cond_t cond;
    pthread_mutex_t mutex;
    pthread_mutex_t request_mutex;
    pthread_mutex_t device_mutex;
    uint32_t pending_sequence;
    uint32_t ack_sequence;
    uint32_t ack_session;
    int32_t ack_cmd;
    int32_t ack_data;
    int pending;
} RPPortDevice;

#define RP_PORT_NUMBER 7
#define RP_DEVICE_PORT_COUNT 6
#define RP_HUB_DEVICE_INDEX 6

extern int raspike_is_valid_port(RasPikePort port);
extern RPPortDevice *getDevice(RasPikePort port);
extern int raspike_prot_lock_status(void);
extern int raspike_prot_unlock_status(void);
extern int raspike_prot_send(RasPikePort port, unsigned char cmdid,
                             const unsigned char *buf, int size);
extern int raspike_prot_send_priority(RasPikePort port, unsigned char cmdid,
                                      const unsigned char *buf, int size);
extern int raspike_request(RasPikePort port, unsigned char cmd,
                           const unsigned char *data, int size,
                           int timeout_ms);
extern int raspike_wait_port_cmd_change(RasPikePort port, unsigned char wait_cmd);
extern int raspike_port_com_change_if_needed(RasPikePort port, unsigned char wait_cmd);
extern RPProtocolSpikeStatus *raspike_prot_get_saved_status(void);

#define GET_DEVICE_COMMON(device_t)                                           \
    RasPikePort r_port = PORT_TO_RASPIKE(port);                               \
    if (!raspike_is_valid_port(r_port) || r_port == RP_PORT_NONE) {            \
        return (device_t *)0;                                                  \
    }                                                                          \
    RPPortDevice *dev = getDevice(r_port);                                     \
    if (!dev) return (device_t *)0;                                            \
    pthread_mutex_lock(&dev->device_mutex);                                    \
    if (dev->device.device_type == fgDeviceType) {                             \
        pthread_mutex_unlock(&dev->device_mutex);                              \
        return (device_t *)&dev->device;                                       \
    }                                                                          \
    if (dev->device.device_type != 0) {                                        \
        printf("RASPIKE_DEVICE_CONFLICT,port=%c,current=%u,requested=%u\n",    \
               port, dev->device.device_type, fgDeviceType);                   \
        pthread_mutex_unlock(&dev->device_mutex);                              \
        return (device_t *)0;                                                  \
    }                                                                          \
    char config_cmd = MAKE_CMD(fgDeviceType, 0);                               \
    int config_result = raspike_request(r_port, config_cmd, 0, 0, 1500);       \
    if (config_result != 1) {                                                  \
        printf("RASPIKE_DEVICE_CONFIG_FAILED,port=%c,type=%u,result=%d\n",    \
               port, fgDeviceType, config_result);                             \
        pthread_mutex_unlock(&dev->device_mutex);                              \
        return (device_t *)0;                                                  \
    }                                                                          \
    dev->device.device_type = fgDeviceType;                                    \
    dev->device.port_id = r_port;                                              \
    pthread_mutex_unlock(&dev->device_mutex);                                  \
    return (device_t *)&dev->device


#define DELAYED_SENSOR_COMMON(ret_type, cmd)                                   \
    ret_type zero_value = {0};                                                 \
    if (!(pdev) || !raspike_is_valid_port((pdev)->port_id)) return zero_value; \
    RasPikePort port = (pdev)->port_id;                                        \
    if (raspike_port_com_change_if_needed(port, (cmd)) != 0) return zero_value;\
    RPProtocolSpikeStatus *p = raspike_prot_get_saved_status();                \
    if (!p) return zero_value;                                                 \
    ret_type result_value;                                                     \
    raspike_prot_lock_status();                                                \
    memcpy(&result_value, p->ports[port].data, sizeof(result_value));          \
    raspike_prot_unlock_status();                                              \
    return result_value

#define GET_AND_RET_SENSOR_COMMON(ret_type, index)                             \
    ret_type zero_value = {0};                                                 \
    if (!(pdev) || !raspike_is_valid_port((pdev)->port_id)) return zero_value; \
    RPProtocolSpikeStatus *p = raspike_prot_get_saved_status();                \
    if (!p) return zero_value;                                                 \
    RPProtocolPortStatus *pp = p->ports + (pdev)->port_id;                     \
    ret_type result_value;                                                     \
    raspike_prot_lock_status();                                                \
    memcpy(&result_value, pp->data + (index), sizeof(result_value));           \
    raspike_prot_unlock_status();                                              \
    return result_value


#ifdef __cplusplus
}
#endif
#endif
