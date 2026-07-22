#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "color.h"
#include "raspike_protocol_api.h"
#include "raspike_protocol_com.h"
#include "raspike_internal.h"
#include "error.h"
#include "button.h"
#include "battery.h"
#include "speaker.h"

static int read_status_snapshot(RPProtocolSpikeStatus *snapshot)
{
    if (!snapshot) return -EINVAL;
    RPProtocolSpikeStatus *status = raspike_prot_get_saved_status();
    if (!status) {
        memset(snapshot, 0, sizeof(*snapshot));
        return -ENOTCONN;
    }
    raspike_prot_lock_status();
    memcpy(snapshot, status, sizeof(*snapshot));
    raspike_prot_unlock_status();
    return 0;
}

static pbio_error_t send_hub_command(unsigned char command,
                                     const void *data, size_t size)
{
    if (size > RP_V2_MAX_PAYLOAD) return PBIO_ERROR_INVALID_ARG;
    int result = raspike_prot_send_priority(
        RP_PORT_NONE, command, (const unsigned char *)data, (int)size);
    if (result >= 0) return PBIO_SUCCESS;
    if (result == -EINVAL) return PBIO_ERROR_INVALID_ARG;
    if (result == -ENOTCONN) return PBIO_ERROR_NO_DEV;
    return PBIO_ERROR_IO;
}

uint16_t hub_battery_get_voltage(void)
{
    RPProtocolSpikeStatus status;
    return read_status_snapshot(&status) == 0 ? status.voltage : 0;
}

uint16_t hub_battery_get_current(void)
{
    RPProtocolSpikeStatus status;
    return read_status_snapshot(&status) == 0 ? status.current : 0;
}

pbio_error_t hub_button_is_pressed(hub_button_t *pressed)
{
    if (!pressed) return PBIO_ERROR_INVALID_ARG;
    RPProtocolSpikeStatus status;
    if (read_status_snapshot(&status) != 0) {
        *pressed = 0;
        return PBIO_ERROR_NO_DEV;
    }
    *pressed = status.button;
    return PBIO_SUCCESS;
}

pbio_error_t hub_display_orientation(uint8_t up)
{
    return send_hub_command(RP_CMD_ID_HUB_DISP_ORI, &up, sizeof(up));
}

pbio_error_t hub_display_off(void)
{
    return send_hub_command(RP_CMD_ID_HUB_DISP_OFF, NULL, 0);
}

pbio_error_t hub_display_pixel(uint8_t row, uint8_t column, uint8_t brightness)
{
    unsigned char data[3] = {row, column, brightness};
    return send_hub_command(RP_CMD_ID_HUB_DISP_PIX, data, sizeof(data));
}

pbio_error_t hub_display_image(uint8_t *image)
{
    if (!image) return PBIO_ERROR_INVALID_ARG;
    return send_hub_command(RP_CMD_ID_HUB_DISP_IMG, image, 25);
}

pbio_error_t hub_display_number(const int8_t num)
{
    return send_hub_command(RP_CMD_ID_HUB_DISP_NUM, &num, sizeof(num));
}

pbio_error_t hub_display_char(const char c)
{
    return send_hub_command(RP_CMD_ID_HUB_DISP_CHR, &c, sizeof(c));
}

pbio_error_t hub_display_text(const char *text, uint32_t on, uint32_t off)
{
    if (!text) return PBIO_ERROR_INVALID_ARG;
    enum { PREFIX = RP_HUB_DISP_TXT_INDEX_TXT };
    unsigned char data[RP_V2_MAX_PAYLOAD] = {0};
    const size_t max_chars = sizeof(data) - PREFIX - 1;
    size_t text_length = strnlen(text, max_chars + 1);
    if (text_length > max_chars) return PBIO_ERROR_INVALID_ARG;
    memcpy(data + RP_HUB_DISP_TXT_INDEX_ON, &on, sizeof(on));
    memcpy(data + RP_HUB_DISP_TXT_INDEX_OFF, &off, sizeof(off));
    memcpy(data + PREFIX, text, text_length);
    data[PREFIX + text_length] = '\0';
    return send_hub_command(RP_CMD_ID_HUB_DISP_TXT,
                            data, PREFIX + text_length + 1);
}

pbio_error_t hub_display_text_scroll(const char *text, uint32_t delay)
{
    if (!text) return PBIO_ERROR_INVALID_ARG;
    enum { PREFIX = RP_HUB_DISP_TXT_SCR_INDEX_TXT };
    unsigned char data[RP_V2_MAX_PAYLOAD] = {0};
    const size_t max_chars = sizeof(data) - PREFIX - 1;
    size_t text_length = strnlen(text, max_chars + 1);
    if (text_length > max_chars) return PBIO_ERROR_INVALID_ARG;
    memcpy(data + RP_HUB_DISP_TXT_SCR_INDEX_DLY, &delay, sizeof(delay));
    memcpy(data + PREFIX, text, text_length);
    data[PREFIX + text_length] = '\0';
    return send_hub_command(RP_CMD_ID_HUB_DISP_TXT_SCR,
                            data, PREFIX + text_length + 1);
}

pbio_error_t hub_imu_init(void)
{
    return PBIO_SUCCESS;
}

void hub_imu_get_acceleration(float accel[3])
{
    if (!accel) return;
    RPProtocolSpikeStatus status;
    if (read_status_snapshot(&status) != 0) {
        memset(accel, 0, sizeof(float) * 3);
        return;
    }
    memcpy(accel, status.acceleration, sizeof(status.acceleration));
}

void hub_imu_get_angular_velocity(float angular_velocity[3])
{
    if (!angular_velocity) return;
    RPProtocolSpikeStatus status;
    if (read_status_snapshot(&status) != 0) {
        memset(angular_velocity, 0, sizeof(float) * 3);
        return;
    }
    memcpy(angular_velocity, status.angular_velocity,
           sizeof(status.angular_velocity));
}

float hub_imu_get_temperature(void)
{
    return 0.0f;
}

bool hub_imu_is_ready(void)
{
    RPProtocolSpikeStatus status;
    return read_status_snapshot(&status) == 0 && status.is_ready;
}

bool hub_imu_is_stationary(void)
{
    RPProtocolSpikeStatus status;
    return read_status_snapshot(&status) == 0 && status.is_statinary;
}

void hub_imu_set_tilt(float angle)
{
    (void)send_hub_command(RP_CMD_ID_HUB_IMU_SET_TLT, &angle, sizeof(angle));
}

float hub_imu_get_heading(void)
{
    RPProtocolSpikeStatus status;
    return read_status_snapshot(&status) == 0 ? status.heading : 0.0f;
}

void hub_imu_reset_heading(void)
{
    (void)send_hub_command(RP_CMD_ID_HUB_IMU_RST_HDG, NULL, 0);
}

pbio_error_t hub_light_on_hsv(const pbio_color_hsv_t *hsv)
{
    if (!hsv) return PBIO_ERROR_INVALID_ARG;
    return send_hub_command(RP_CMD_ID_HUB_LGT_ON_HSV, hsv, sizeof(*hsv));
}

pbio_error_t hub_light_on_color(pbio_color_t color)
{
    return send_hub_command(RP_CMD_ID_HUB_LGT_ON_COL, &color, sizeof(color));
}

pbio_error_t hub_light_off(void)
{
    return send_hub_command(RP_CMD_ID_HUB_LGT_OFF, NULL, 0);
}

void hub_speaker_set_volume(uint8_t volume)
{
    if (volume > 100) volume = 100;
    (void)send_hub_command(RP_CMD_ID_HUB_SPK_SET_VOL, &volume, sizeof(volume));
}

void hub_speaker_play_tone(uint16_t frequency, int32_t duration)
{
    unsigned char data[6] = {0};
    const int32_t remote_duration = SOUND_MANUAL_STOP;
    memcpy(data + RP_HUB_SPK_PLY_TON_INDEX_DUR,
           &remote_duration, sizeof(remote_duration));
    memcpy(data + RP_HUB_SPK_PLY_TON_INDEX_FRQ,
           &frequency, sizeof(frequency));
    if (send_hub_command(RP_CMD_ID_HUB_SPK_PLY_TON,
                         data, sizeof(data)) != PBIO_SUCCESS) {
        return;
    }

    if (duration >= 0) {
        struct timespec request = {
            .tv_sec = duration / 1000,
            .tv_nsec = (long)(duration % 1000) * 1000000L,
        };
        struct timespec remaining;
        while (nanosleep(&request, &remaining) != 0 && errno == EINTR) {
            request = remaining;
        }
        hub_speaker_stop();
    }
}

void hub_speaker_stop(void)
{
    (void)send_hub_command(RP_CMD_ID_HUB_SPK_STP, NULL, 0);
}

void hub_system_restart(void)
{
    const uint32_t magic = RP_SOFT_RESET_MAGIC;
    (void)raspike_request(RP_PORT_NONE, RP_CMD_ID_SOFT_RST,
                          (const unsigned char *)&magic, sizeof(magic), 2000);
}

void hub_system_enter_update_mode(void)
{
    const uint32_t magic = RP_UPDATE_MODE_MAGIC;
    (void)raspike_request(RP_PORT_NONE, RP_CMD_ID_UPDATE_MODE,
                          (const unsigned char *)&magic, sizeof(magic), 2000);
}

void hub_system_shutdown(void)
{
    (void)send_hub_command(RP_CMD_ID_SHT_DWN, NULL, 0);
    raspike_prot_shutdown();
}
