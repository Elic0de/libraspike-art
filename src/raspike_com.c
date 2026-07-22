#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "raspike_com.h"

#define RP_COM_TYPE_USB 1
#define RP_SEND_BUFFER_SIZE 4096
#define RP_DEFAULT_IO_TIMEOUT_MS 1000

#define LOGF(...) do { fprintf(stdout, __VA_ARGS__); fflush(stdout); } while (0)

typedef int (*RPComSendFunc)(RPComDescriptor *desc, const unsigned char *buf, int size);
typedef int (*RPComReceiveFunc)(RPComDescriptor *desc, unsigned char *buf, int size, int timeout_ms);
typedef int (*RPComCloseFunc)(RPComDescriptor *desc);
typedef int (*RPComFlushFunc)(RPComDescriptor *desc);

struct _RPComDescriptor {
    int type;
    int desc;
    int io_timeout_ms;
    RPComSendFunc send;
    RPComReceiveFunc receive;
    RPComCloseFunc close;
    RPComFlushFunc flush;
};

static unsigned char fg_send_buffer[RP_SEND_BUFFER_SIZE];
static size_t fg_send_buffer_size;
static int fg_send_mode = RASPIKE_USB_MODE_NORMAL;
static pthread_mutex_t fg_send_mutex = PTHREAD_MUTEX_INITIALIZER;

static int64_t monotonic_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static int get_usb_fd(RPComDescriptor *desc)
{
    if (!desc || desc->type != RP_COM_TYPE_USB) {
        errno = EINVAL;
        return -1;
    }
    return desc->desc;
}

static int wait_fd(int fd, short events, int timeout_ms)
{
    struct pollfd pfd = {.fd = fd, .events = events, .revents = 0};
    int ret;
    do {
        ret = poll(&pfd, 1, timeout_ms);
    } while (ret < 0 && errno == EINTR);

    if (ret == 0) return -ETIMEDOUT;
    if (ret < 0) return -errno;
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) return -EIO;
    return 0;
}

static int write_all_fd(int fd, const unsigned char *buf, size_t size, int timeout_ms)
{
    if (!buf && size) return -EINVAL;
    size_t written = 0;
    int64_t deadline = monotonic_ms() + (timeout_ms > 0 ? timeout_ms : RP_DEFAULT_IO_TIMEOUT_MS);

    while (written < size) {
        int remaining_ms = (int)(deadline - monotonic_ms());
        if (remaining_ms <= 0) return -ETIMEDOUT;
        int ready = wait_fd(fd, POLLOUT, remaining_ms);
        if (ready < 0) return ready;

        ssize_t len = write(fd, buf + written, size - written);
        if (len > 0) {
            written += (size_t)len;
            continue;
        }
        if (len < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        return len == 0 ? -EIO : -errno;
    }
    return (int)written;
}

static int read_some_fd(int fd, unsigned char *buf, size_t size, int timeout_ms)
{
    if (!buf || size == 0) return -EINVAL;
    int ready = wait_fd(fd, POLLIN, timeout_ms);
    if (ready < 0) return ready;

    for (;;) {
        ssize_t len = read(fd, buf, size);
        if (len > 0) return (int)len;
        if (len == 0) return -EPIPE;
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return -EAGAIN;
        return -errno;
    }
}

static int buffered_write(const unsigned char *buf, size_t size)
{
    if ((!buf && size) || size > RP_SEND_BUFFER_SIZE) return -EINVAL;
    pthread_mutex_lock(&fg_send_mutex);
    if (fg_send_buffer_size + size > sizeof(fg_send_buffer)) {
        pthread_mutex_unlock(&fg_send_mutex);
        return -ENOBUFS;
    }
    memcpy(fg_send_buffer + fg_send_buffer_size, buf, size);
    fg_send_buffer_size += size;
    pthread_mutex_unlock(&fg_send_mutex);
    return (int)size;
}

static int flush_buffer_locked(RPComDescriptor *desc)
{
    if (fg_send_buffer_size == 0) return 0;
    int fd = get_usb_fd(desc);
    if (fd < 0) return -EINVAL;

    int ret = write_all_fd(fd, fg_send_buffer, fg_send_buffer_size, desc->io_timeout_ms);
    if (ret < 0) return ret;
    fg_send_buffer_size = 0;
    return ret;
}

void raspike_usb_buffer_flush(RPComDescriptor *desc)
{
    pthread_mutex_lock(&fg_send_mutex);
    int ret = flush_buffer_locked(desc);
    pthread_mutex_unlock(&fg_send_mutex);
    if (ret < 0) LOGF("RASPIKE_LINK_ERROR,operation=flush,error=%d\n", -ret);
}

void raspike_usb_set_mode(int mode)
{
    pthread_mutex_lock(&fg_send_mutex);
    fg_send_mode = mode == RASPIKE_USB_MODE_BUFFERED
        ? RASPIKE_USB_MODE_BUFFERED
        : RASPIKE_USB_MODE_NORMAL;
    pthread_mutex_unlock(&fg_send_mutex);
}

static int rp_usb_send(RPComDescriptor *desc, const unsigned char *buf, int size)
{
    if (size < 0) return -EINVAL;
    pthread_mutex_lock(&fg_send_mutex);
    int mode = fg_send_mode;
    pthread_mutex_unlock(&fg_send_mutex);
    if (mode == RASPIKE_USB_MODE_BUFFERED) return buffered_write(buf, (size_t)size);

    int fd = get_usb_fd(desc);
    if (fd < 0) return -EINVAL;
    return write_all_fd(fd, buf, (size_t)size, desc->io_timeout_ms);
}

static int rp_usb_receive(RPComDescriptor *desc, unsigned char *buf, int size, int timeout_ms)
{
    if (size <= 0) return -EINVAL;
    int fd = get_usb_fd(desc);
    if (fd < 0) return -EINVAL;
    return read_some_fd(fd, buf, (size_t)size, timeout_ms);
}

static int rp_usb_close(RPComDescriptor *desc)
{
    int fd = get_usb_fd(desc);
    if (fd < 0) return -EINVAL;
    int ret = close(fd);
    desc->desc = -1;
    return ret == 0 ? 0 : -errno;
}

static int rp_usb_flush(RPComDescriptor *desc)
{
    pthread_mutex_lock(&fg_send_mutex);
    int ret = flush_buffer_locked(desc);
    pthread_mutex_unlock(&fg_send_mutex);
    return ret < 0 ? ret : 0;
}

RPComDescriptor *raspike_open_usb_communication(const char *device_name)
{
    if (!device_name) return NULL;
    int fd = open(device_name, O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK);
    if (fd < 0) {
        LOGF("RASPIKE_LINK_ERROR,operation=open,error=%d,device=%s\n", errno, device_name);
        return NULL;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        int saved = errno;
        close(fd);
        errno = saved;
        return NULL;
    }
    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= CLOCAL | CREAD | CS8;
    tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        int saved = errno;
        close(fd);
        errno = saved;
        return NULL;
    }
    tcflush(fd, TCIOFLUSH);

    RPComDescriptor *desc = calloc(1, sizeof(*desc));
    if (!desc) {
        close(fd);
        return NULL;
    }
    desc->type = RP_COM_TYPE_USB;
    desc->desc = fd;
    desc->io_timeout_ms = RP_DEFAULT_IO_TIMEOUT_MS;
    desc->send = rp_usb_send;
    desc->receive = rp_usb_receive;
    desc->close = rp_usb_close;
    desc->flush = rp_usb_flush;
    return desc;
}


int raspike_com_discard_input(RPComDescriptor *desc)
{
    int fd = get_usb_fd(desc);
    if (fd < 0) return -EINVAL;
    if (tcflush(fd, TCIFLUSH) != 0) return -errno;
    return 0;
}

int raspike_com_send(RPComDescriptor *desc, const unsigned char *buf, int size)
{
    if (!desc || !desc->send) return -EINVAL;
    int ret = desc->send(desc, buf, size);
    if (ret < 0) LOGF("RASPIKE_LINK_ERROR,operation=send,error=%d\n", -ret);
    return ret;
}

int raspike_com_receive_timeout(RPComDescriptor *desc, unsigned char *buf, int size, int timeout_ms)
{
    if (!desc || !desc->receive || !buf || size < 0) return -EINVAL;
    if (size == 0) return 0;

    int received = 0;
    int64_t deadline = monotonic_ms() + (timeout_ms > 0 ? timeout_ms : RP_DEFAULT_IO_TIMEOUT_MS);
    while (received < size) {
        int remaining_ms = (int)(deadline - monotonic_ms());
        if (remaining_ms <= 0) return -ETIMEDOUT;
        int len = desc->receive(desc, buf + received, size - received, remaining_ms);
        if (len > 0) {
            received += len;
            continue;
        }
        if (len == -EAGAIN || len == -EINTR) continue;
        return len;
    }
    return received;
}

int raspike_com_receive(RPComDescriptor *desc, unsigned char *buf, int size)
{
    return raspike_com_receive_timeout(desc, buf, size, RP_DEFAULT_IO_TIMEOUT_MS);
}

int raspike_com_close(RPComDescriptor *desc)
{
    if (!desc) return -EINVAL;
    int ret = desc->close ? desc->close(desc) : 0;
    free(desc);
    return ret;
}

int raspike_com_flush(RPComDescriptor *desc)
{
    if (!desc || !desc->flush) return -EINVAL;
    return desc->flush(desc);
}
