/*
 * EM11 -- EuroMISS controller with USB interface
 *
 * Copyright (C) 2015 Alexey Filin
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * The header file defines command layer of BCU protocol stack for EuroMISS:
 *
 *   * addresses of EM11 registers,
 *   * EuroMISS work modes,
 *   * virtual funcs to encode & decode EM11 commands
 *
 * One or some EuroMISS commands packed in a buffer are executed as a request
 * to controller. Request can contain one or more register operations and is
 * executed as a single whole. No intermediate state can be saved and
 * no execution can be continued later.
 * 
 */
#ifndef _BCU_EM11_H_
#define _BCU_EM11_H_ 1

#ifdef __cplusplus
extern "C" {
#endif

struct bcu_dev;
struct _bcu_dev_vt;

enum {
  BCU_EM11_RESET_WORD = 0x8000, /* status register word to reset EuroMISS */
};

/* addresses of EM11 registers */
typedef enum bcu_em11_reg_addr {
  BCU_EM11_ADDR_STATUS     = 0x0,  /* status register */
  BCU_EM11_ADDR_READ_CMD   = 0x1,  /* read command register */
  BCU_EM11_ADDR_WRITE_CMD  = 0x2,  /* write command register */
  BCU_EM11_ADDR_EMISS_MODE = 0x3,  /* MISS work mode and registration control register */
  BCU_EM11_ADDR_DATA       = 0x4,  /* data register */
  BCU_EM11_ADDR_SSON_AB    = 0x5,  /* register to read address buffer (AZU), set sync signal on */
  BCU_EM11_ADDR_SSOFF_DB   = 0x6,  /* register to read data buffer (DZU), set sync signal off */
  BCU_EM11_ADDR_SR_STATUS  = 0x7,  /* register to get status of sequential data/address read */
} bcu_em11_reg_addr_t;

/* EuroMISS work modes */
typedef enum bcu_em11_emiss_mode {
  BCU_EM11_MODE_ADDR  = (0x0 << 6),  /* address mode */
  BCU_EM11_MODE_SNR   = (0x1 << 6),  /* sequential number read */
  BCU_EM11_MODE_SDR   = (0x2 << 6),  /* sequential data read */
  BCU_EM11_MODE_AUTO  = (0x3 << 6),  /* autonomous mode */
} bcu_em11_mode_t;

/* Controller work mode codes */
//#define BCU_EM11_CODE_TEST  0x0  /* test mode */
//#define BCU_EM11_CODE_WORK  0x1  /* work mode */

/* Initialize EM11 instance
   \param pointer to device instance

   NB: struct bcu_dev should be inited with the function before any use */
int bcu_em11_init( struct bcu_dev * );

/* get pointer to virtual funcs table */
struct _bcu_dev_vt *bcu_em11_get_vt( void );

#ifdef __cplusplus
}
#endif

#endif /* _BCU_EM11_CMD_H_ */
