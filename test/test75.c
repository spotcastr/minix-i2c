/* Test 75 - i2c /dev interface
 * Don't know if this belongs in test, but I needed a quick and easy way
 * to cross compile a test program. 
 */

#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <dev/i2c/i2c_io.h>

#include "common.h"

void
do_write(void)
{
	int fd, r;
	i2c_ioctl_exec_t ioctl_exec;
	char buf[32];

	memset(&ioctl_exec, '\0', sizeof(i2c_ioctl_exec_t));
	memset(buf, '\0', 32);

	buf[0] = 0x00; /* memory address to start writing to */
	buf[1] = 0x04;

	buf[2] = 'x';
	buf[3] = '3';

	ioctl_exec.iie_op = I2C_OP_WRITE_WITH_STOP;
	ioctl_exec.iie_addr = 0x50; /* EEPROM with board info */
	ioctl_exec.iie_cmd = (void *) NULL;
	ioctl_exec.iie_cmdlen = 0;
	ioctl_exec.iie_buf = (void *) buf;
	ioctl_exec.iie_buflen = 4; /* read 8 bytes */

	fd = open("/dev/i2c-2", O_RDWR);
	if (fd == -1) {
		e(1);
		return;
	}

	r = ioctl(fd, I2C_IOCTL_EXEC, &ioctl_exec);
	if (r == -1) {
		e(5);
		return;
	}

	r = close(fd);
	if (r == -1) {
		e(10);
		return;
	}

	memset(&ioctl_exec, '\0', sizeof(i2c_ioctl_exec_t));
	memset(buf, '\0', 32);

	buf[0] = 0x00; /* memory address to start writing to */
	buf[1] = 0x00;

	buf[2] = 'm';
	buf[3] = 'i';
	buf[4] = 'n';
	buf[5] = 'i';

	ioctl_exec.iie_op = I2C_OP_WRITE_WITH_STOP;
	ioctl_exec.iie_addr = 0x50; /* EEPROM with board info */
	ioctl_exec.iie_cmd = (void *) NULL;
	ioctl_exec.iie_cmdlen = 0;
	ioctl_exec.iie_buf = (void *) buf;
	ioctl_exec.iie_buflen = 6; /* read 8 bytes */

	fd = open("/dev/i2c-2", O_RDWR);
	if (fd == -1) {
		e(1);
		return;
	}

	r = ioctl(fd, I2C_IOCTL_EXEC, &ioctl_exec);
	if (r == -1) {
		e(5);
		return;
	}

	r = close(fd);
	if (r == -1) {
		e(10);
		return;
	}

}

void
do_read(void)
{
	int fd, r;
	i2c_ioctl_exec_t ioctl_exec;
	char cmd[2];
	char buf[32];
	char msg[7];

	memset(&ioctl_exec, '\0', sizeof(i2c_ioctl_exec_t));
	memset(cmd, '\0', 2); /* CMD of 0x00 0x00 reads from addr 0x00 */
	memset(buf, '\0', 32);

	/* Read 8 bytes starting at 0x04 (board name) */

	cmd[0] = 0x00; /* memory address to start reading from */
	cmd[1] = 0x00;

	ioctl_exec.iie_op = I2C_OP_READ_WITH_STOP;
	ioctl_exec.iie_addr = 0x50; /* EEPROM with board info */
	ioctl_exec.iie_cmd = (void *) cmd;
	ioctl_exec.iie_cmdlen = 2;
	ioctl_exec.iie_buf = (void *) buf;
	ioctl_exec.iie_buflen = 6; /* read 8 bytes */

	fd = open("/dev/i2c-2", O_RDWR);
	if (fd == -1) {
		e(1);
		return;
	}

	r = ioctl(fd, I2C_IOCTL_EXEC, &ioctl_exec);
	if (r == -1) {
		e(5);
		return;
	}

	r = close(fd);
	if (r == -1) {
		e(10);
		return;
	}

	memset(msg, '\0', 7);
	memcpy(msg, buf, 6);

	printf("msg='%s'\n", msg);
}

int
main(int argc, char *argv[])
{
	start(75);

	do_write();
	do_read();

	quit();
	return 0;
}

