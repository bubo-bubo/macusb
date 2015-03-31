/*
 * LE98 -- MISS controller with USB interface
 *
 * Copyright (C) 2014 Alexey Filin
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * The header file defines command layer of BCU protocol stack for MISS:
 *
 *   * addresses of LE98 registers,
 *   * MISS work modes,
 *   * LE98 commands
 *   * funcs to encode, execute, decode LE98 commands, wrap macusb driver
 *     ioctl's
 *
 * One or some MISS commands packed in a buffer are executed as a request to
 * controller. Request can contain one or more register operations and is
 * executed as a single whole. No intermediate state can be saved and
 * no execution can be continued later.
 * 
 */
#ifndef _BCU_LE98_H_
#define _BCU_LE98_H_ 1

#include <sys/types.h>
#include "macusb.h"

#ifdef __cplusplus
extern "C" {
#endif

/* addresses of LE98 registers */
typedef enum bcu_le98_reg_addr {
  BCU_LE98_STATUS_ADDR    = 0x0,  /* status register */
  BCU_LE98_CMD_ADDR       = 0x1,  /* command register */
  BCU_LE98_TEST_ADDR      = 0x2,  /* test register */
  BCU_LE98_MISS_MODE_ADDR = 0x3,  /* MISS work mode and registration control register */
  BCU_LE98_DATA_ADDR      = 0x4,  /* data register */
  BCU_LE98_SYNC_ADDR      = 0x5,  /* sync register */
} bcu_le98_reg_addr_t;

/* MISS work modes */
typedef enum bcu_le98_miss_mode {
  BCU_LE98_MISS_ADDR_MODE  = (0x0 << 6),  /* address mode */
  BCU_LE98_MISS_SNR_MODE   = (0x1 << 6),  /* sequential number read */
  BCU_LE98_MISS_SDR_MODE   = (0x2 << 6),  /* sequential data read */
  BCU_LE98_MISS_AUTO_MODE  = (0x3 << 6),  /* autonomous mode */
  BCU_LE98_MISS_RSV_MODE   = (0x4 << 6),  /* reserved */
  BCU_LE98_MISS_AFSNR_MODE = (0x5 << 6),  /* asynchronous fast sequential number read */
  BCU_LE98_MISS_AFSDR_MODE = (0x6 << 6),  /* asynchronous fast sequential data read */
  BCU_LE98_MISS_SFSDR_MODE = (0x7 << 6),  /* synchronous fast sequential data read */
} bcu_le98_miss_mode_t;

/* Controller work mode codes */
//#define BCU_LE98_TEST_MODE  0x0  /* test mode */
//#define BCU_LE98_WORK_MODE  0x1  /* work mode */

/* LE98 commands */
typedef enum bcu_le98_cmd {
  BCU_LE98_INIT_CTL_CMD = 0, /* init controller, see LE98 description */
  BCU_LE98_MODE_GET_CMD,     /* get work mode */
  BCU_LE98_MODE_SET_CMD,     /* set work mode */
  BCU_LE98_REG_READ_CMD,     /* read controller register */
  BCU_LE98_REG_WRITE_CMD,    /* write to controller register */
  BCU_LE98_NAF_READ_CMD,     /* MISS read NAF */
  BCU_LE98_NAF_WRITE_CMD,    /* MISS write NAF */
  BCU_LE98_NAF_CONTROL_CMD,  /* MISS control NAF */
/* Sequential Number Read (SNR) and Sequential Data Read (SDR) modes suppose
   command execution dependent from each other in crate controller so commands
   below should be encoded into empty TX buffer */
  BCU_LE98_SNR_START_CMD,    /* start PChN */
  BCU_LE98_SNR_READ_CMD,     /* read PChN */
  BCU_LE98_SNR_STOP_CMD,     /* stop PChN */
  BCU_LE98_SDR_START_CMD,    /* start PChI */
  BCU_LE98_SDR_READ_CMD,     /* read PChI */
  BCU_LE98_SDR_STOP_CMD      /* stop PChI */
} bcu_le98_cmd_t;

/* size of path buffer */
#define BCU_LE98_PATH_BUFSIZE  256

/* transfer flags */
typedef enum bcu_le98_transfer_flags {
  BCU_LE98_SYNC_MODE = 1, /* synchronous transfer */
} bcu_le98_transfer_flags_t;

/* default transfer_flags */
#define BCU_LE98_TANSFER_FLAGS  0

/* LE98 device node representation */
typedef struct bcu_le98 {
  int          id;             /* device id */
  char        *dev_dir;        /* location of LE98 device files */
  char         path[BCU_LE98_PATH_BUFSIZE]; /* path to device file */
  int          fd;             /* device file descriptor */
  unsigned int transfer_flags; /* bit OR of transfer flags */
} bcu_le98_t;

/* Initialize LE98 instance
   \param pointer to device instance

   NB: bcu_le98_t should be inited with the function before any use */
void bcu_le98_init( bcu_le98_t * );

/* Finalize LE98 instance
   \param pointer to device instance
   \return 0 -- success, <0 -- syscall error */
int bcu_le98_fini( bcu_le98_t * );

/* Open LE98 device file by DeviceID
   \param pointer to device instance
   \param LE98 DeviceID
   \return 0 -- success, >0 -- wrong args, <0 -- syscall error

   NB: If some LE98 with the same ID are used bcu_le98_open_path() is to be
   used with extra device-path mapping logic */
int bcu_le98_open_id( bcu_le98_t *, unsigned int );

/* Open LE98 device file by path
   \param pointer to device instance
   \param path to device file
   \return 0 -- success, >0 -- wrong path, <0 -- syscall error

   NB: bcu_le98_t.id is set to -1. Path is copied */
int bcu_le98_open_path( bcu_le98_t *, const char * );

/* Close LE98 device file
   \param pointer to device instance
   \return >=0 -- device file descriptor, <0 -- syscall error */
int bcu_le98_close( bcu_le98_t * );

/* Get device file status
   \param pointer to device instance
   \param pointer to get device file status
   \return 0 -- success, <0 -- syscall error */
int bcu_le98_status( bcu_le98_t *, macusb_status_t * );

/* Reset usb device
   \param pointer to device instance
   \return 0 -- success, <0 -- syscall error

   NB: LE98 device file is to be reopen after the operation */
int bcu_le98_reset( bcu_le98_t * );

/* Reset controller
   \param pointer to device instance
   \return 0 -- success, <0 -- syscall error

   NB: LE98 device file is to be reopen after the operation */
int bcu_le98_reset_ctl( bcu_le98_t * );

/* Clear device
   \param pointer to device instance
   \return 0 -- success, <0 -- syscall error */
int bcu_le98_clear( bcu_le98_t * );

/* Get configuration for synchronous transfers
   \param pointer to device instance
   \param pointer to get synchronous configuration
   \return >=0 -- device file descriptor, <0 -- syscall error */
int bcu_le98_get_sync_cfg( bcu_le98_t *, macusb_sync_cfg_t * );

/* Set configuration for synchronous transfers
   \param pointer to device instance
   \param pointer to set synchronous configuration
   \return >=0 -- device file descriptor, <0 -- syscall error */
int bcu_le98_set_sync_cfg( bcu_le98_t *, macusb_sync_cfg_t * );

/* Get configuration for asynchronous transfers
   \param pointer to device instance
   \param pointer to get asynchronous configuration
   \return >=0 -- device file descriptor, <0 -- syscall error */
int bcu_le98_get_async_cfg( bcu_le98_t *, macusb_async_cfg_t * );

/* Set configuration for asynchronous transfers
   \param pointer to device instance
   \param pointer to set asynchronous configuration
   \return >=0 -- device file descriptor, <0 -- syscall error */
int bcu_le98_set_async_cfg( bcu_le98_t *, macusb_async_cfg_t * );

/* Execute command
   \param pointer to device instance
   \param command
   \param pointer to store state by START_SNR, READ_SNR, START_SDR and READ_SDR commands, ignored by other commands
   \param pointer to store NAF by MISS commands, address by READ_REG, READ_SNR and READ_SDR commands, ignored by other commands
   \param pointer to store read data by READ_REG, READ_NAF and READ_SDR commands, ignored by other commands
   \return 0 -- success, >0 -- controller/decoding error, <0 -- syscall error */
int bcu_le98_exec_cmd( bcu_le98_t *, bcu_le98_cmd_t, u_int16_t *, u_int16_t *, u_int16_t * );

/* Execute command buffer
   \param pointer to device instance
   \param command buffer
   \return 0 -- success, <0 -- syscall error */
int bcu_le98_exec_buf( bcu_le98_t *, macusb_buffer_t * );

/* Encode command
   \param command buffer
   \param command
   \param register address for READ_REG and WRITE_REG, NAF for CONTROL_NAF, READ_NAF and WRITE_NAF commands, ignored by other commands
   \param data to write
   \return 0 -- success, >0 -- encoding error

   NB: the function doesn't change data size in TX buffer if failed
 */
int bcu_le98_encode_cmd( macusb_buffer_t *, bcu_le98_cmd_t, u_int16_t, u_int16_t );

/* Decode command
   \param command buffer
   \param pointer to store decoding offset
   \param command
   \param pointer to store state by START_SNR, READ_SNR, START_SDR and READ_SDR commands, ignored by other commands
   \param pointer to store NAF by CONTROL_NAF, READ_NAF and WRITE_NAF commands, address by READ_SNR and READ_SDR commands, ignored by other commands
   \param pointer to store read data by READ_REG, READ_NAF and READ_SDR commands, ignored by other commands
   \return 0 -- success, >0 -- controller/decoding error

   NB: Offset is shifted to the next command if decoding succedded or points to the packet decoded with error */
int bcu_le98_decode_cmd( macusb_buffer_t *, unsigned int *, bcu_le98_cmd_t, u_int16_t *, u_int16_t *, u_int16_t * );

/* Get device node state description
   \param device node state
   \return description or NULL if no such state */
const char *bcu_le98_state_str( unsigned int );

#ifdef __cplusplus
}
#endif

#endif /* _BCU_LE98_CMD_H_ */
