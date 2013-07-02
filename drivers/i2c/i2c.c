/*
 * i2c - generic driver for Inter-Integrated Circuit bus (I2C).
 */

/* kernel headers */
#include <minix/chardriver.h>
#include <minix/drivers.h>
#include <minix/ds.h>
#include <minix/i2c.h>
#include <minix/log.h>
#include <minix/type.h>

/* system headers */
#include <sys/mman.h>

/* usr headers */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* SoC specific headers */
#include "omap_i2c.h"

/* local function prototypes */
static int validate_ioctl_exec(minix_i2c_ioctl_exec_t * ioctl_exec);
static int do_i2c_ioctl_exec(message * m);

/* libchardriver callbacks */
int i2c_ioctl(message * m);
struct device *i2c_prepare(dev_t dev);
int i2c_transfer(endpoint_t endpt, int opcode, u64_t position,
    iovec_t * iov, unsigned nr_req, endpoint_t user_endpt, unsigned int flags);
int i2c_other(message * m);

/* SEF callbacks and driver state management */
static int sef_cb_lu_state_save(int);
static int lu_state_restore(void);
static int sef_cb_init(int type, sef_init_info_t * info);
static void sef_local_startup(void);

/* Globals  */

/* the bus that this instance of the driver is responsible for */
uint32_t i2c_bus_id;

/* Process a request for an i2c operation */
int (*process) (minix_i2c_ioctl_exec_t * ioctl_exec);

struct device i2c_device;

/* logging - use with log_warn(), log_info(), log_debug(), log_trace() */
static struct log log = {
	.name = "i2c",
	.log_level = LEVEL_INFO,
	.log_func = default_log
};

/* Entry points to the i2c driver. */
static struct chardriver i2c_tab = {
	.cdr_open = do_nop,
	.cdr_close = do_nop,
	.cdr_ioctl = i2c_ioctl,
	.cdr_prepare = i2c_prepare,
	.cdr_transfer = i2c_transfer,
	.cdr_cleanup = nop_cleanup,
	.cdr_alarm = nop_alarm,
	.cdr_cancel = nop_cancel,
	.cdr_select = nop_select,
	.cdr_other = i2c_other
};

/*
 * Checks a minix_i2c_ioctl_exec_t to see if the fields make sense.
 */
static int
validate_ioctl_exec(minix_i2c_ioctl_exec_t * ioctl_exec)
{
	i2c_op_t op;
	size_t len;

	op = ioctl_exec->iie_op;
	if (op != I2C_OP_READ &&
	    op != I2C_OP_READ_WITH_STOP &&
	    op != I2C_OP_WRITE &&
	    op != I2C_OP_WRITE_WITH_STOP &&
	    op != I2C_OP_READ_BLOCK && op != I2C_OP_WRITE_BLOCK) {
		log_warn(&log, "invalid value for iie_op\n");
		return EINVAL;
	}

	len = ioctl_exec->iie_cmdlen;
	if (len < 0 || len > I2C_EXEC_MAX_CMDLEN) {
		log_warn(&log,
		    "iie_cmdlen out of range 0-I2C_EXEC_MAX_CMDLEN\n");
		return EINVAL;
	}

	len = ioctl_exec->iie_buflen;
	if (len < 0 || len > I2C_EXEC_MAX_BUFLEN) {
		log_warn(&log,
		    "iie_buflen out of range 0-I2C_EXEC_MAX_BUFLEN\n");
		return EINVAL;
	}

	return OK;
}

/*
 * Performs the action in minix_i2c_ioctl_exec_t.
 */
static int
do_i2c_ioctl_exec(message * m)
{
	int r;
	endpoint_t caller;
	cp_grant_id_t grant_nr;
	minix_i2c_ioctl_exec_t ioctl_exec;

	caller = (endpoint_t) m->m_source;
	grant_nr = (cp_grant_id_t) m->IO_GRANT;

	/* Copy the requested exection into the driver */
	r = sys_safecopyfrom(caller, grant_nr, (vir_bytes) 0,
	    (vir_bytes) & ioctl_exec, sizeof(ioctl_exec));
	if (r != OK) {
		log_warn(&log, "sys_safecopyfrom() failed\n");
		return r;
	}

	/* input validation */
	r = validate_ioctl_exec(&ioctl_exec);
	if (r != OK) {
		return r;
	}

	r = process(&ioctl_exec);
	if (r != OK) {
		return r;
	}

	/* Copy the results of the execution back to the calling process */
	r = sys_safecopyto(caller, grant_nr, (vir_bytes) 0,
	    (vir_bytes) & ioctl_exec, sizeof(ioctl_exec));
	if (r != OK) {
		log_warn(&log, "sys_safecopyto() failed\n");
		return r;
	}

	return OK;
}

int
i2c_ioctl(message * m)
{
	int r;

	switch (m->COUNT) {
	case MINIX_I2C_IOCTL_EXEC:
		r = do_i2c_ioctl_exec(m);
		break;
	default:
		log_warn(&log, "Invalid ioctl()\n");
		r = EINVAL;
		break;
	}

	return r;
}

int
i2c_other(message * m)
{
	int r;

	switch (m->m_type) {
	case BUSC_I2C_EXEC:
		m->COUNT = MINIX_I2C_IOCTL_EXEC;
		r = do_i2c_ioctl_exec(m);
		break;
	case NOTIFY_MESSAGE:
		r = OK;
		break;
	default:
		log_warn(&log, "Invalid message type (%d)\n", m->m_type);
		r = EINVAL;
		break;
	}

	return r;
}

struct device *
i2c_prepare(dev_t dev)
{
	i2c_device.dv_base = make64(0, 0);
	i2c_device.dv_size = make64(0, 0);

	return &i2c_device;
}

int
i2c_transfer(endpoint_t endpt, int opcode, u64_t position,
    iovec_t * iov, unsigned nr_req, endpoint_t user_endpt, unsigned int flags)
{
	return OK;
}

static int
sef_cb_lu_state_save(int UNUSED(state))
{
	ds_publish_u32("i2c_bus_id", i2c_bus_id, DSF_OVERWRITE);
	return OK;
}

static int
lu_state_restore(void)
{
	ds_retrieve_u32("i2c_bus_id", &i2c_bus_id);
	ds_delete_u32("i2c_bus_id");

	return OK;
}

static int
sef_cb_init(int type, sef_init_info_t * UNUSED(info))
{
	int r;
	int do_announce_driver = TRUE;

	switch (type) {

	case SEF_INIT_FRESH:
		/* Select the correct i2c Implementation for this SoC */
#if defined(AM335X) || defined(DM37XX)
		r = omap_interface_setup(&process, i2c_bus_id);
		if (r != OK) {
			return r;
		}
#else
#error				/* Unknown SoC or bad configuration */
#endif
		break;

	case SEF_INIT_LU:
		/* Restore the state. */
		lu_state_restore();
		do_announce_driver = FALSE;
		break;

	case SEF_INIT_RESTART:
		break;
	}

	/* Announce we are up when necessary. */
	if (do_announce_driver) {
		chardriver_announce();
	}

	/* Initialization completed successfully. */
	return OK;
}

static void
sef_local_startup()
{
	/* Register init callbacks. */
	sef_setcb_init_fresh(sef_cb_init);
	sef_setcb_init_lu(sef_cb_init);
	sef_setcb_init_restart(sef_cb_init);

	/* Register live update callbacks */
	/* Agree to update immediately when LU is requested in a valid state */
	sef_setcb_lu_prepare(sef_cb_lu_prepare_always_ready);
	/* - Support live update starting from any standard state */
	sef_setcb_lu_state_isvalid(sef_cb_lu_state_isvalid_standard);
	/* - Register a custom routine to save the state. */
	sef_setcb_lu_state_save(sef_cb_lu_state_save);

	/* Let SEF perform startup. */
	sef_startup();
}

int
main(int argc, char *argv[])
{
	int r, ipc_status;
	long instance;
	message i2c_mess;

	env_setargs(argc, argv);

	/* Parse the instance number passed to service */
	instance = 0;
	r = env_parse("instance", "d", 0, &instance, 1, 3);
	if (r == -1) {
		fprintf(stderr,
		    "Expecting '-arg instance=N' argument (N=1..3)\n");
		return EXIT_FAILURE;
	}

	/* Device files count from 1, hardware starts counting from 0 */
	i2c_bus_id = instance - 1;

	sef_local_startup();
	chardriver_task(&i2c_tab, CHARDRIVER_SYNC);

	return OK;
}
