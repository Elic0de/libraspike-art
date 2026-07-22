#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "raspike_protocol_api.h"
#include "raspike_protocol_com.h"
#include "raspike_internal.h"
#include "motor.h"

DECLARE_DEVICE_TYPE_IN_FILE(RP_CMD_TYPE_MOTOR);

static pbio_error_t request_result_to_pbio(int result)
{
    if (result == 1 || result == PBIO_SUCCESS) return PBIO_SUCCESS;
    if (result == -ETIMEDOUT) return PBIO_ERROR_TIMEDOUT;
    if (result == -EINVAL) return PBIO_ERROR_INVALID_ARG;
    if (result == -ENODEV || result == -ENOTCONN) return PBIO_ERROR_NO_DEV;
    return PBIO_ERROR_IO;
}

pup_motor_t *pup_motor_get_device(pbio_port_id_t port)
{
    GET_DEVICE_COMMON(pup_motor_t);
}

pbio_error_t pup_motor_setup(pup_motor_t *motor,
                             pup_direction_t positive_direction,
                             bool reset_count)
{
    if (!motor) return PBIO_ERROR_INVALID_ARG;
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    unsigned char data[RP_MOTOR_STU_INDEX_RESETCOUNT + sizeof(bool)] = {0};
    memcpy(data + RP_MOTOR_STU_INDEX_DIRECTION,
           &positive_direction, sizeof(positive_direction));
    memcpy(data + RP_MOTOR_STU_INDEX_RESETCOUNT,
           &reset_count, sizeof(reset_count));
    int result = raspike_request(pdev->port_id, RP_CMD_ID_MOT_STU,
                                 data, sizeof(data), 2000);
    return request_result_to_pbio(result);
}

pbio_error_t pup_motor_reset_count(pup_motor_t *motor)
{
    if (!motor) return PBIO_ERROR_INVALID_ARG;
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    int result = raspike_request(pdev->port_id, RP_CMD_ID_MOT_RST,
                                 NULL, 0, 2000);
    if (result == 1) {
        RPProtocolSpikeStatus *status = raspike_prot_get_saved_status();
        if (status) {
            raspike_prot_lock_status();
            memset(status->ports[pdev->port_id].data + RP_MOTOR_INDEX_COUNT,
                   0, sizeof(int32_t));
            raspike_prot_unlock_status();
        }
    }
    return request_result_to_pbio(result);
}

int32_t pup_motor_get_count(pup_motor_t *motor)
{
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    GET_AND_RET_SENSOR_COMMON(int32_t, RP_MOTOR_INDEX_COUNT);
}

int32_t pup_motor_get_speed(pup_motor_t *motor)
{
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    GET_AND_RET_SENSOR_COMMON(int32_t, RP_MOTOR_INDEX_SPEED);
}

pbio_error_t pup_motor_set_speed(pup_motor_t *motor, int speed)
{
    if (!motor) return PBIO_ERROR_INVALID_ARG;
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    pdev->cmd = RP_CMD_ID_MOT_SPD;
    int sent = raspike_prot_send(pdev->port_id, RP_CMD_ID_MOT_SPD,
                                 (const unsigned char *)&speed, sizeof(speed));
    return sent < 0 ? PBIO_ERROR_IO : PBIO_SUCCESS;
}

int32_t pup_motor_get_power(pup_motor_t *motor)
{
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    if (!pdev) return 0;
    if (pdev->cmd == RP_CMD_ID_MOT_POW) return pdev->power;
    GET_AND_RET_SENSOR_COMMON(int16_t, RP_MOTOR_INDEX_POWER);
}

pbio_error_t pup_motor_set_power(pup_motor_t *motor, int power)
{
    if (!motor) return PBIO_ERROR_INVALID_ARG;
    if (power < -100) power = -100;
    if (power > 100) power = 100;
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    pdev->cmd = RP_CMD_ID_MOT_POW;
    pdev->power = power;
    int sent = raspike_prot_send(pdev->port_id, RP_CMD_ID_MOT_POW,
                                 (const unsigned char *)&power, sizeof(power));
    return sent < 0 ? PBIO_ERROR_IO : PBIO_SUCCESS;
}

pbio_error_t pup_motor_stop(pup_motor_t *motor)
{
    if (!motor) return PBIO_ERROR_INVALID_ARG;
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    return request_result_to_pbio(raspike_request(
        pdev->port_id, RP_CMD_ID_MOT_STP, NULL, 0, 1000));
}

pbio_error_t pup_motor_brake(pup_motor_t *motor)
{
    if (!motor) return PBIO_ERROR_INVALID_ARG;
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    return request_result_to_pbio(raspike_request(
        pdev->port_id, RP_CMD_ID_MOT_STP_BRK, NULL, 0, 1000));
}

pbio_error_t pup_motor_hold(pup_motor_t *motor)
{
    if (!motor) return PBIO_ERROR_INVALID_ARG;
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    return request_result_to_pbio(raspike_request(
        pdev->port_id, RP_CMD_ID_MOT_STP_HLD, NULL, 0, 1000));
}

bool pup_motor_is_stalled(pup_motor_t *motor)
{
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    GET_AND_RET_SENSOR_COMMON(bool, RP_MOTOR_INDEX_ISSTALLED);
}

int32_t pup_motor_set_duty_limit(pup_motor_t *motor, int duty_limit)
{
    if (!motor) return -EINVAL;
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    return raspike_request(pdev->port_id, RP_CMD_ID_MOT_SET_DTY,
                           (const unsigned char *)&duty_limit,
                           sizeof(duty_limit), 1000);
}

void pup_motor_restore_duty_limit(pup_motor_t *motor, int old_value)
{
    if (!motor) return;
    struct _pup_device_t *pdev = (struct _pup_device_t *)motor;
    (void)raspike_request(pdev->port_id, RP_CMD_ID_MOT_RST_DTY,
                          (const unsigned char *)&old_value,
                          sizeof(old_value), 1000);
}
