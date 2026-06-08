#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "raspike_com.h"
#include "raspike_protocol_api.h"
#include "spike/hub/imu.h"
#include "spike/pup/colorsensor.h"
#include "spike/pup/motor.h"

#define DEFAULT_DEVICE "/dev/USB_SPIKE"
#define DEFAULT_ITERATIONS 2000
#define DEFAULT_PERIOD_US 10000
#define DEFAULT_PWM 0

typedef enum {
  MEASURE_MODE_MOTOR,
  MEASURE_MODE_ALL,
  MEASURE_MODE_TRACE,
} MeasureMode;

static volatile int g_receive_running = 1;

static uint64_t monotonic_us(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static void sleep_until_us(uint64_t deadline_us)
{
  for (;;) {
    uint64_t now_us = monotonic_us();
    if (now_us >= deadline_us) {
      return;
    }
    uint64_t remaining_us = deadline_us - now_us;
    struct timespec req;
    req.tv_sec = (time_t)(remaining_us / 1000000ULL);
    req.tv_nsec = (long)((remaining_us % 1000000ULL) * 1000ULL);
    while (nanosleep(&req, &req) != 0) {
      if (errno != EINTR) {
        return;
      }
    }
  }
}

static void *receiver_task(void *arg)
{
  (void)arg;
  while (g_receive_running) {
    raspike_prot_receive();
  }
  return NULL;
}

static int port_id(const char *name)
{
  if (name == NULL || strlen(name) != 1 || name[0] < 'A' || name[0] > 'F') {
    fprintf(stderr, "port must be A-F: %s\n", name == NULL ? "(null)" : name);
    exit(2);
  }
  return name[0] - 'A';
}

static int parse_int_arg(const char *value, const char *label)
{
  char *end = NULL;
  long parsed = strtol(value, &end, 10);
  if (end == value || *end != '\0') {
    fprintf(stderr, "%s must be an integer: %s\n", label, value);
    exit(2);
  }
  return (int)parsed;
}

static int clamp_power(int value)
{
  if (value > 100) return 100;
  if (value < -100) return -100;
  return value;
}

static MeasureMode parse_mode_arg(const char *value)
{
  if (strcmp(value, "motor") == 0) {
    return MEASURE_MODE_MOTOR;
  }
  if (strcmp(value, "all") == 0) {
    return MEASURE_MODE_ALL;
  }
  if (strcmp(value, "trace") == 0) {
    return MEASURE_MODE_TRACE;
  }
  fprintf(stderr, "mode must be motor, all, or trace: %s\n", value);
  exit(2);
}

static void usage(const char *argv0)
{
  fprintf(stderr,
          "usage: %s [device] [iterations] [period_us] [pwm] [left_port] [right_port] [mode] [color_port]\n"
          "defaults: device=%s iterations=%d period_us=%d pwm=%d left=B right=A mode=motor color=E\n"
          "modes: motor=fixed PWM, all=fixed PWM plus sensors, trace=color/gyro correction plus variable PWM\n",
          argv0, DEFAULT_DEVICE, DEFAULT_ITERATIONS, DEFAULT_PERIOD_US, DEFAULT_PWM);
}

int main(int argc, char const *argv[])
{
  const char *device = DEFAULT_DEVICE;
  int iterations = DEFAULT_ITERATIONS;
  int period_us = DEFAULT_PERIOD_US;
  int pwm = DEFAULT_PWM;
  int left_port = port_id("B");
  int right_port = port_id("A");
  MeasureMode mode = MEASURE_MODE_MOTOR;
  int color_port = port_id("E");

  if (argc > 9) {
    usage(argv[0]);
    return 2;
  }
  if (argc > 1) device = argv[1];
  if (argc > 2) iterations = parse_int_arg(argv[2], "iterations");
  if (argc > 3) period_us = parse_int_arg(argv[3], "period_us");
  if (argc > 4) pwm = parse_int_arg(argv[4], "pwm");
  if (argc > 5) left_port = port_id(argv[5]);
  if (argc > 6) right_port = port_id(argv[6]);
  if (argc > 7) mode = parse_mode_arg(argv[7]);
  if (argc > 8) color_port = port_id(argv[8]);

  if (iterations <= 1 || period_us <= 0 || pwm < -100 || pwm > 100) {
    usage(argv[0]);
    return 2;
  }
  setenv("RASPIKE_MEASURE_RX_CSV", "spike_usb_loop_period.csv", 0);

  RPComDescriptor *desc = raspike_open_usb_communication(device);
  if (desc == NULL) {
    fprintf(stderr, "cannot open device: %s\n", device);
    return 1;
  }
  if (raspike_prot_init(desc) != 0) {
    fprintf(stderr, "raspike_prot_init failed\n");
    return 1;
  }

  pthread_t receiver;
  pthread_create(&receiver, NULL, receiver_task, NULL);

  pup_motor_t *left = pup_motor_get_device((pbio_port_id_t)left_port);
  pup_motor_t *right = pup_motor_get_device((pbio_port_id_t)right_port);
  pup_motor_setup(left, PUP_DIRECTION_COUNTERCLOCKWISE, true);
  pup_motor_setup(right, PUP_DIRECTION_CLOCKWISE, true);

  pup_device_t *color = NULL;
  if (mode == MEASURE_MODE_ALL || mode == MEASURE_MODE_TRACE) {
    hub_imu_init();
    color = pup_color_sensor_get_device((pbio_port_id_t)color_port);
    if (mode == MEASURE_MODE_TRACE) {
      (void)pup_color_sensor_reflection(color);
    } else {
      (void)pup_color_sensor_rgb(color);
    }
  }

  fprintf(stdout,
          "source,seq,timestamp_us,dt_us,body_us,deadline_lag_us,left_pwm,right_pwm,"
          "left_count,right_count,left_speed,right_speed,heading,"
          "accel_x,accel_y,accel_z,angv_x,angv_y,angv_z,color_r,color_g,color_b,"
          "color_reflection,correction\n");
  fflush(stdout);

  uint64_t next_deadline_us = monotonic_us();
  uint64_t previous_start_us = 0;
  for (int seq = 0; seq < iterations; ++seq) {
    sleep_until_us(next_deadline_us);
    uint64_t start_us = monotonic_us();
    int64_t deadline_lag_us = start_us >= next_deadline_us
                                   ? (int64_t)(start_us - next_deadline_us)
                                   : -(int64_t)(next_deadline_us - start_us);
    uint64_t dt_us = previous_start_us == 0 ? 0 : start_us - previous_start_us;
    previous_start_us = start_us;

    int left_pwm = pwm;
    int right_pwm = pwm;
    int32_t left_count = 0;
    int32_t right_count = 0;
    int32_t left_speed = 0;
    int32_t right_speed = 0;
    int32_t reflection = 0;
    int correction = 0;
    float heading = 0.0f;
    float accel[3] = {0.0f, 0.0f, 0.0f};
    float angv[3] = {0.0f, 0.0f, 0.0f};
    pup_color_rgb_t rgb = {0};

    if (mode == MEASURE_MODE_ALL || mode == MEASURE_MODE_TRACE) {
      left_count = pup_motor_get_count(left);
      right_count = pup_motor_get_count(right);
      left_speed = pup_motor_get_speed(left);
      right_speed = pup_motor_get_speed(right);
      heading = hub_imu_get_heading();
      hub_imu_get_acceleration(accel);
      hub_imu_get_angular_velocity(angv);
      if (mode == MEASURE_MODE_TRACE) {
        reflection = pup_color_sensor_reflection(color);
        correction = (reflection - 50) / 2 + (int)(heading / 4.0f);
        left_pwm = clamp_power(pwm + correction);
        right_pwm = clamp_power(pwm - correction);
      } else {
        rgb = pup_color_sensor_rgb(color);
      }
    }

    pup_motor_set_power(left, left_pwm);
    pup_motor_set_power(right, right_pwm);

    uint64_t end_us = monotonic_us();
    fprintf(stdout,
            "libraspike,%d,%llu,%llu,%llu,%lld,%d,%d,"
            "%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%d\n",
            seq,
            (unsigned long long)start_us,
            (unsigned long long)dt_us,
            (unsigned long long)(end_us - start_us),
            (long long)deadline_lag_us,
            left_pwm,
            right_pwm,
            left_count,
            right_count,
            left_speed,
            right_speed,
            (double)heading,
            (double)accel[0],
            (double)accel[1],
            (double)accel[2],
            (double)angv[0],
            (double)angv[1],
            (double)angv[2],
            rgb.r,
            rgb.g,
            rgb.b,
            reflection,
            correction);

    next_deadline_us += (uint64_t)period_us;
  }

  pup_motor_stop(left);
  pup_motor_stop(right);
  raspike_prot_measure_flush();
  sleep(2);
  g_receive_running = 0;
  pthread_cancel(receiver);
  pthread_join(receiver, NULL);
  raspike_prot_shutdown();
  fflush(stdout);
  return 0;
}
