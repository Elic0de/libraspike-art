#include <errno.h>
#include <stdio.h>

#include "raspike_protocol_api.h"
#include "raspike_protocol_com.h"
#include "raspike_internal.h"
#include "colorsensor.h"

DECLARE_DEVICE_TYPE_IN_FILE(RP_CMD_TYPE_COLOR);

static pbio_error_t request_result_to_pbio(int result)
{
    if (result == 1 || result == PBIO_SUCCESS) return PBIO_SUCCESS;
    if (result == -ETIMEDOUT) return PBIO_ERROR_TIMEDOUT;
    if (result == -EINVAL) return PBIO_ERROR_INVALID_ARG;
    return PBIO_ERROR_IO;
}

pup_device_t *pup_color_sensor_get_device(pbio_port_id_t port)
{
    GET_DEVICE_COMMON(pup_device_t);
}

pup_color_hsv_t pup_color_sensor_color(pup_device_t *pdev, bool surface)
{
    unsigned char cmd = surface ? RP_CMD_ID_COL_COL : RP_CMD_ID_COL_COL_SUR_OFF;
    DELAYED_SENSOR_COMMON(pup_color_hsv_t, cmd);
}

pup_color_rgb_t pup_color_sensor_rgb(pup_device_t *pdev)
{
    DELAYED_SENSOR_COMMON(pup_color_rgb_t, RP_CMD_ID_COL_RGB);
}

pup_color_hsv_t pup_color_sensor_hsv(pup_device_t *pdev, bool surface)
{
    unsigned char cmd = surface ? RP_CMD_ID_COL_HSV : RP_CMD_ID_COL_HSV_SUR_OFF;
    DELAYED_SENSOR_COMMON(pup_color_hsv_t, cmd);
}

int32_t pup_color_sensor_reflection(pup_device_t *pdev)
{
    DELAYED_SENSOR_COMMON(int32_t, RP_CMD_ID_COL_REF);
}

int32_t pup_color_sensor_ambient(pup_device_t *pdev)
{
    DELAYED_SENSOR_COMMON(int32_t, RP_CMD_ID_COL_AMB);
}

pbio_error_t pup_color_sensor_light_set(pup_device_t *pdev,
                                        int32_t bv1, int32_t bv2, int32_t bv3)
{
    if (!pdev) return PBIO_ERROR_INVALID_ARG;
    int32_t data[3] = {bv1, bv2, bv3};
    int result = raspike_request(pdev->port_id, RP_CMD_ID_COL_LIGHT_SET,
                                 (const unsigned char *)data, sizeof(data), 1000);
    return request_result_to_pbio(result);
}

pbio_error_t pup_color_sensor_light_on(pup_device_t *pdev)
{
    if (!pdev) return PBIO_ERROR_INVALID_ARG;
    int sent = raspike_prot_send_priority(pdev->port_id,
                                          RP_CMD_ID_COL_LIGHT_ON, NULL, 0);
    return sent < 0 ? PBIO_ERROR_IO : PBIO_SUCCESS;
}

pbio_error_t pup_color_sensor_light_off(pup_device_t *pdev)
{
    if (!pdev) return PBIO_ERROR_INVALID_ARG;
    int sent = raspike_prot_send_priority(pdev->port_id,
                                          RP_CMD_ID_COL_LIGHT_OFF, NULL, 0);
    return sent < 0 ? PBIO_ERROR_IO : PBIO_SUCCESS;
}

pup_color_hsv_t *pup_color_sensor_detectable_colors(int32_t size,
                                                     pup_color_hsv_t *colors)
{
    (void)size;
    (void)colors;
    return NULL;
}
