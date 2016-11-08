/*
 * LE98 -- MISS controller with USB interface
 *
 * Copyright (C) 2014-2015 Alexey Filin
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * The header file defines command layer of BCU protocol stack for MISS:
 *
 *   * addresses of LE98 registers,
 *   * work modes,
 *   * virtual funcs to encode & decode LE98 commands
 *
 * One or some MISS commands packed in a buffer are executed as a request
 * to controller. Request can contain one or more register operations and is
 * executed as a single whole. No intermediate state can be saved and
 * no execution can be continued later.
 * 
 */
#ifndef _BCU_LE98_H_
#define _BCU_LE98_H_ 1

#ifdef __cplusplus
extern "C" {
#endif

struct bcu_dev;
struct _bcu_dev_vt;

/* addresses of LE98 registers */
typedef enum bcu_le98_reg_addr {
  BCU_LE98_ADDR_STATUS = 0x0,  /* status register */
  BCU_LE98_ADDR_CMD    = 0x1,  /* command register */
  BCU_LE98_ADDR_TEST   = 0x2,  /* test register */
  BCU_LE98_ADDR_MODE   = 0x3,  /* MISS work mode and registration control register */
  BCU_LE98_ADDR_DATA   = 0x4,  /* data register */
  BCU_LE98_ADDR_SYNC   = 0x5,  /* sync register */
} bcu_le98_reg_addr_t;

/* MISS work modes */
typedef enum bcu_le98_mode {
  BCU_LE98_MODE_ADDR  = (0x0 << 6),  /* address mode */
  BCU_LE98_MODE_SNR   = (0x1 << 6),  /* sequential number read */
  BCU_LE98_MODE_SDR   = (0x2 << 6),  /* sequential data read */
  BCU_LE98_MODE_AUTO  = (0x3 << 6),  /* autonomous mode */
  BCU_LE98_MODE_RSV   = (0x4 << 6),  /* reserved */
  BCU_LE98_MODE_AFSNR = (0x5 << 6),  /* asynchronous fast sequential number read */
  BCU_LE98_MODE_AFSDR = (0x6 << 6),  /* asynchronous fast sequential data read */
  BCU_LE98_MODE_SFSDR = (0x7 << 6),  /* synchronous fast sequential data read */
} bcu_le98_mode_t;

/* Controller work mode codes */
//#define BCU_LE98_CODE_TEST  0x0  /* test mode */
//#define BCU_LE98_CODE_WORK  0x1  /* work mode */

/* Initialize LE98 instance
   \param pointer to device instance

   NB: struct bcu_dev should be inited with the function before any use */
int bcu_le98_init( struct bcu_dev * );

/* get pointer to virtual funcs table */
struct _bcu_dev_vt *bcu_le98_get_vt( void );

#ifdef __cplusplus
}
#endif

#endif /* _BCU_LE98_CMD_H_ */
