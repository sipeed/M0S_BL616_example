#ifndef _GSL2038_I2C_H
#define _GSL2038_I2C_H

#include "bflb_core.h"
#include "touch.h"

int gsl2038_i2c_init(touch_coord_t *max_value);
int gsl2038_i2c_read(uint8_t *point_num, touch_coord_t *touch_coord, uint8_t max_num);

#endif /* _GSL2038_I2C_H */
