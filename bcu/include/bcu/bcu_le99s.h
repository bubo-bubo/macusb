/*
 * LE99S -- SUMMA controller with USB interface
 *
 * Copyright (C) 2015 Alexey Filin
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * The header file defines command layer of BCU protocol stack for SUMMA:
 *
 *   * addresses of LE99S registers,
 *   * virtual funcs to encode & decode LE99S commands
 *
 * One or some SUMMA commands packed in a buffer are executed as a request
 * to controller. Request can contain one or more register operations and is
 * executed as a single whole. No intermediate state can be saved and
 * no execution can be continued later.
 * 
 */
#ifndef _BCU_LE99S_H_
#define _BCU_LE99S_H_ 1

#ifdef __cplusplus
extern "C" {
#endif

struct bcu_dev;
struct _bcu_dev_vt;

/* addresses of LE99S registers */
typedef enum bcu_le99s_reg_addr {
  BCU_LE99S_ADDR_COMMAND    = 0x0,  /* command register */
  BCU_LE99S_ADDR_STATUS     = 0x1,  /* status register */
  BCU_LE99S_ADDR_DATA       = 0x4,  /* data register */
  BCU_LE99S_ADDR_LAM1       = 0x5,  /* register to read low LAM lines, send pulse to output 1 */
  BCU_LE99S_ADDR_LAM2       = 0x6,  /* register to read high LAM lines, send pulse to output 2 */
} bcu_le99s_reg_addr_t;

/* Initialize LE99S instance
   \param pointer to device instance

   NB: struct bcu_dev should be inited with the function before any use */
int bcu_le99s_init( struct bcu_dev * );

/* get pointer to virtual funcs table */
struct _bcu_dev_vt *bcu_le99s_get_vt( void );

#ifdef __cplusplus
}
#endif

#endif /* _BCU_LE99S_CMD_H_ */
