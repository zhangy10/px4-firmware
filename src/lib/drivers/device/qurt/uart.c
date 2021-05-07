
#include <px4_log.h>
#include "uart.h"

// Static variables
static bool _callbacks_configured = false;

static open_uart_func_t  _open_uart  = NULL;
static write_uart_func_t _write_uart = NULL;
static read_uart_func_t  _read_uart  = NULL;

void configure_uart_callbacks(open_uart_func_t open_func,
                              write_uart_func_t write_func,
                              read_uart_func_t read_func) {
    _open_uart  = open_func;
    _write_uart = write_func;
    _read_uart  = read_func;
    if (_open_uart && _write_uart && _read_uart) {
        _callbacks_configured = true;
    }
}

int qurt_uart_open(const char *dev, speed_t speed)
{
    if (_callbacks_configured) {
        // Convert device string into a uart port number
        char *endptr = NULL;
        uint8_t port_number = (uint8_t) strtol(dev, &endptr, 10);
        if ((port_number == 0) && (endptr == dev)) {
            PX4_ERR("Could not convert %s into a valid uart port number", dev);
            return -1;
        }
    	return _open_uart(port_number, speed);
    } else {
        PX4_ERR("Cannot open uart until callbacks have been configured");
    }

    return -1;
}

int qurt_uart_write(int fd, const char *buf, size_t len)
{
	if (fd < 0) {
		PX4_ERR("invalid fd %d for %s", fd, __FUNCTION__);
		return -1;
	}

    if (buf == NULL) {
		PX4_ERR("NULL buffer pointer in %s", fd, __FUNCTION__);
		return -1;
    }

    if (len == 0) {
		PX4_ERR("Zero length buffer in %s", __FUNCTION__);
		return -1;
    }

    if (_callbacks_configured) {
        return _write_uart(fd, buf, len);
    } else {
        PX4_ERR("Cannot write to uart until callbacks have been configured");
    }

    return -1;
}

int qurt_uart_read(int fd, char *buf, size_t len)
{
	if (fd < 0) {
		PX4_ERR("invalid fd %d for %s", fd, __FUNCTION__);
		return -1;
	}

    if (buf == NULL) {
		PX4_ERR("NULL buffer pointer in %s", fd, __FUNCTION__);
		return -1;
    }

    if (len == 0) {
		PX4_ERR("Zero length buffer in %s", __FUNCTION__);
		return -1;
    }

    if (_callbacks_configured) {
        return _read_uart(fd, buf, len);
    } else {
        PX4_ERR("Cannot read from uart until callbacks have been configured");
    }

    return -1;
}
