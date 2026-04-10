#ifndef __I2C_H
#define __I2C_H

uint32_t rtlsdr_get_tuner_clock(void *dev);
int rtlsdr_i2c_write_fn(void *dev, uint8_t addr, uint8_t *buf, int len);
int rtlsdr_i2c_read_fn(void *dev, uint8_t addr, uint8_t *buf, int len);

/* V4 support — dongle model identification.
 * Used by tuner_r82xx.c for Blog V4 input switching. */
int rtlsdr_check_dongle_model(void *dev, const char *manufact_check, const char *product_check);

/* Dongle info accessor — fills caller-provided buffers with human-readable
 * dongle identification.  Used by class_driver.c (which can't see the
 * rtlsdr_dev struct internals).  Any output pointer may be NULL to skip. */
void rtlsdr_get_dongle_info(void *dev, char *model_out, int model_len,
                            int *tuner_type_out, int *force_bt_out);

#endif