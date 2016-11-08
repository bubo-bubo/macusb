/*
 * Bus controller with USB interface (MACUSB device)
 *
 * Copyright (C) 2015-2016 Alexey Filin
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * The header file defines convenience wrappers for ioctl's with MACUSB device.
 *
 */
#ifndef _BCU_DEV_H_
#define _BCU_DEV_H_ 1

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* size of path buffer */
#define BCU_DEV_PATH_BUFSIZE  256

/* transfer flags */
typedef enum bcu_dev_transfer_flags {
  BCU_DEV_FLAG_SYNC = 1, /* synchronous transfer */
} bcu_dev_transfer_flags_t;

/* default transfer_flags */
#define BCU_DEV_TANSFER_FLAGS  0

/* BCU commands */
typedef enum bcu_dev_cmd {
  BCU_DEV_CMD_INIT_CTL = 0, /* init controller */
  BCU_DEV_CMD_MODE_GET,     /* get work mode */
  BCU_DEV_CMD_MODE_SET,     /* set work mode */
  BCU_DEV_CMD_REG_READ,     /* read controller register */
  BCU_DEV_CMD_REG_WRITE,    /* write to controller register */
  BCU_DEV_CMD_NAF_READ,     /* MISS read NAF */
  BCU_DEV_CMD_NAF_WRITE,    /* MISS write NAF */
  BCU_DEV_CMD_NAF_CONTROL,  /* MISS control NAF */
/* Sequential Number Read (SNR aka PChN) and Sequential Data Read
   (SDR aka PChI) modes suppose command execution dependent from each other
   in crate controller so commands below should be encoded into empty
   TX buffer */
  BCU_DEV_CMD_SNR_START,    /* start PChN */
  BCU_DEV_CMD_SNR_READ,     /* PChN read */
  BCU_DEV_CMD_SNR_STOP,     /* PChN stop */
  BCU_DEV_CMD_SDR_START,    /* start PChI */
  BCU_DEV_CMD_SDR_READ,     /* PChI read */
  BCU_DEV_CMD_SDR_STOP,     /* PChI stop */
  BCU_DEV_CMD_SR_STATUS,    /* read PCh status */
} bcu_dev_cmd_t;

struct macusb_buffer;
struct macusb_status;
struct macusb_sync_cfg;
struct macusb_async_cfg;

/* table of virtual functions */
typedef struct _bcu_dev_vt {
  int (*encode_cmd)( void *, struct macusb_buffer *, unsigned int, uint16_t, uint16_t );
  int (*decode_cmd)( void *, struct macusb_buffer *, unsigned int *, unsigned int, uint16_t *, uint16_t *, uint16_t * );
  char *name;
} _bcu_dev_vt_t;

/* device node representation */
typedef struct bcu_dev {
  int          id;             /* device id */
  char        *path;           /* path to device node */
  int          fd;             /* device node descriptor */
  unsigned int transfer_flags; /* bit OR of transfer flags */

  _bcu_dev_vt_t _vt; /* virtual funcs table */
} bcu_dev_t;

/* Initialize BCU instance
   \param pointer to device instance
   \return 0 -- success, <0 -- syscall error

   NB: bcu_dev_t should be inited with the function before any use */
int bcu_dev_init( bcu_dev_t * );

/* Finalize BCU instance
   \param pointer to device instance
   \return 0 -- success, <0 -- syscall error */
int bcu_dev_fini( bcu_dev_t * );

/* Open BCU device file by DeviceID
   \param pointer to device instance
   \param BCU DeviceID
   \return 0 -- success, >0 -- wrong args, <0 -- syscall error

   NB: If some BCU with the same ID are used bcu_dev_open_path() is to be
   used with extra device-path mapping logic */
int bcu_dev_open_id( void *, unsigned int );

/* Open BCU device file by path
   \param pointer to device instance
   \param path to device file
   \return 0 -- success, >0 -- wrong path, <0 -- syscall error

   NB: bcu_dev_t.id is set to -1. Path is copied */
int bcu_dev_open_path( void *, const char * );

/* Close BCU device file
   \param pointer to device instance
   \return >=0 -- device file descriptor, <0 -- syscall error */
int bcu_dev_close( void * );

/* Get device file status
   \param pointer to device instance
   \param pointer to get device file status
   \return 0 -- success, <0 -- syscall error */
int bcu_dev_status( void *, struct macusb_status * );

/* Reset usb device
   \param pointer to device instance
   \return 0 -- success, <0 -- syscall error

   NB: BCU device file is to be reopen after the operation */
int bcu_dev_reset( void * );

/* Reset controller
   \param pointer to device instance
   \return 0 -- success, <0 -- syscall error

   NB: BCU device file is to be reopen after the operation */
int bcu_dev_reset_ctl( void * );

/* Clear device
   \param pointer to device instance
   \return 0 -- success, <0 -- syscall error */
int bcu_dev_clear( void * );

/* Get configuration for synchronous transfers
   \param pointer to device instance
   \param pointer to get synchronous configuration
   \return >=0 -- device file descriptor, <0 -- syscall error */
int bcu_dev_get_sync_cfg( void *, struct macusb_sync_cfg * );

/* Set configuration for synchronous transfers
   \param pointer to device instance
   \param pointer to set synchronous configuration
   \return >=0 -- device file descriptor, <0 -- syscall error */
int bcu_dev_set_sync_cfg( void *, struct macusb_sync_cfg * );

/* Get configuration for asynchronous transfers
   \param pointer to device instance
   \param pointer to get asynchronous configuration
   \return >=0 -- device file descriptor, <0 -- syscall error */
int bcu_dev_get_async_cfg( void *, struct macusb_async_cfg * );

/* Set configuration for asynchronous transfers
   \param pointer to device instance
   \param pointer to set asynchronous configuration
   \return >=0 -- device file descriptor, <0 -- syscall error */
int bcu_dev_set_async_cfg( void *, struct macusb_async_cfg * );

/* Execute command
   \param pointer to device instance
   \param command
   \param pointer to save state word
   \param pointer to save attributes (e.g. address)
   \param pointer to save data
   \return 0 -- success, <0 -- syscall error */
int bcu_dev_exec_cmd( void *self, unsigned int cmd, uint16_t *state, uint16_t *attr, uint16_t *data );

/* Execute command buffer
   \param pointer to device instance
   \param command buffer
   \return 0 -- success, <0 -- syscall error */
int bcu_dev_exec_buf( void *self, struct macusb_buffer *buf );

int bcu_dev_encode_cmd( void *self, struct macusb_buffer *buf, unsigned int cmd, uint16_t attr, uint16_t data );

int bcu_dev_decode_cmd( void *self, struct macusb_buffer *buf, unsigned int *offset, unsigned int cmd, uint16_t *state, uint16_t *attr, uint16_t *data );

/* Get device node state description
   \param device node state
   \return description or NULL if no such state */
const char *bcu_dev_state_str( unsigned int );

/* get pointer to virtual funcs table */
_bcu_dev_vt_t *bcu_dev_get_vt( void );

#ifdef __cplusplus
}
#endif

#endif /* _BCU_DEV_CMD_H_ */
