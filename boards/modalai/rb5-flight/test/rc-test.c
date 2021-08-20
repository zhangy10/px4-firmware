#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <stdint.h>


int main() {
	int uart_fd = open("/dev/ttyHS1", O_RDWR | O_NONBLOCK);

    if (uart_fd < 0) {
        fprintf(stderr, "Error: couldn't open serial port\n");
        return -1;
    }

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(uart_fd, &uart_config);

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, B115200)) < 0) {
		fprintf(stderr, "Error: %d (cfsetispeed)", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, B115200)) < 0) {
		fprintf(stderr, "Error: %d (cfsetospeed)", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
	    fprintf(stderr, "Error: %d (tcsetattr)", termios_state);
		return -1;
	}

    int loop_counter = 0;
	uint8_t rx_buf[32];
    int newbytes = 0;
    while (loop_counter < 25) {
        newbytes = read(uart_fd, &rx_buf[0], sizeof(rx_buf));
        if (newbytes > 0) break;
        usleep(100000);
        loop_counter++;
    }

    close(uart_fd);

    if (newbytes > 0) {
        printf("Read %d bytes on loop %d\n", newbytes, loop_counter);
        return 0;
    } else fprintf(stderr, "Error: failed to read from RC port\n");

    return -1;
}
