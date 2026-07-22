#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "raspike_protocol_api.h"
#include "raspike_protocol_com.h"
#include "raspike_internal.h"
#include "ultrasonicsensor.h"

DECLARE_DEVICE_TYPE_IN_FILE(RP_CMD_TYPE_US);

pup_device_t *pup_ultrasonic_sensor_get_device(pbio_port_id_t port)
{
    GET_DEVICE_COMMON(pup_device_t);
}

int32_t pup_ultrasonic_sensor_distance(pup_device_t *pdev)
{
    GET_AND_RET_SENSOR_COMMON(int32_t, RP_US_INDEX_DISTANCE);
}

bool pup_ultrasonic_sensor_presence(pup_device_t *pdev)
{
    GET_AND_RET_SENSOR_COMMON(bool, RP_US_INDEX_PRESENCE);
}

pbio_error_t pup_ultrasonic_sensor_light_set(pup_device_t *pdev,
                                             int32_t bv1, int32_t bv2,
                                             int32_t bv3, int32_t bv4)
{
    if (!pdev) return PBIO_ERROR_INVALID_ARG;
    unsigned char data[16] = {0};
    memcpy(data + RP_US_LGT_SET_INDEX_BV1, &bv1, sizeof(bv1));
    memcpy(data + RP_US_LGT_SET_INDEX_BV2, &bv2, sizeof(bv2));
    memcpy(data + RP_US_LGT_SET_INDEX_BV3, &bv3, sizeof(bv3));
    memcpy(data + RP_US_LGT_SET_INDEX_BV4, &bv4, sizeof(bv4));
    int result = raspike_prot_send_priority(
        pdev->port_id, RP_CMD_ID_US_LGT_SET, data, sizeof(data));
    return result < 0 ? PBIO_ERROR_IO : PBIO_SUCCESS;
}

pbio_error_t pup_ultrasonic_sensor_light_on(pup_device_t *pdev)
{
    if (!pdev) return PBIO_ERROR_INVALID_ARG;
    int result = raspike_prot_send_priority(
        pdev->port_id, RP_CMD_ID_US_LGT_ON, NULL, 0);
    return result < 0 ? PBIO_ERROR_IO : PBIO_SUCCESS;
}

pbio_error_t pup_ultrasonic_sensor_light_off(pup_device_t *pdev)
{
    if (!pdev) return PBIO_ERROR_INVALID_ARG;
    int result = raspike_prot_send_priority(
        pdev->port_id, RP_CMD_ID_US_LGT_OFF, NULL, 0);
    return result < 0 ? PBIO_ERROR_IO : PBIO_SUCCESS;
}
