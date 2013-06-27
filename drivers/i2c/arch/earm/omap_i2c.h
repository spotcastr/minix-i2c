#ifndef _OMAP_I2C_H
#define _OMAP_I2C_H

#include <minix/chardriver.h>
#include "omap_i2c_registers.h"

#ifndef __ASSEMBLY__

int omap_interface_setup(int (**process)(minix_i2c_ioctl_exec_t *ioctl_exec), int i2c_bus_id);

#endif /* !__ASSEMBLY__ */

#endif /* _OMAP_I2C_H */
