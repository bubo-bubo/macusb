/*
 * MAC USB driver
 *
 * Copyright (C) 2014-2015 Alexey Filin
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * The driver implements request layer of Bus controller - USB (BCU) protocol
 * stack.
 *
 * Request layer:
 *
 *   * Implements data exchange with USB pipes.
 *
 *   * Provides 15 usec average latency of request execution with
 *     Mechanism of Accelerated Completion (MAC).
 *     URB completion handler is called on microframe edges (microframe
 *     duration is 125 usec) so a lower limit of average execution time is
 *     about 63 usec. MAC is a bypass of URB completion handler.
 *
 * Request is bound one-for-one with a set of two URBs:
 *
 *   1. TX URB (request transmission, bulk-out URB)
 *   2. RX URB (answer receiving, bulk-in URB)
 *
 * Request transmission is implemented with one TX URB with a data block in TX
 * buffer encoded completely by upper layer in BCU protocol stack.
 * No limits on format of data block are imposed by request layer.
 *
 * Answer receiving is implemented with one RX URB with a data block in RX
 * buffer ended with 4-byte trailer decoded by request layer. The trailer is
 * used by MAC. Received data block is to be decoded by upper layer in BCU
 * protocol stack.
 * 
 * The driver implements access to a MAC USB device with character
 * special file from devfs (device node). Each device attached to USB
 * and enumerated is assigned with a unique device node name. The name matches
 * pattern /dev/macusb_TYPE_N-X, where:
 * 
 *   o TYPE is a device type, string defined by idProduct in the device default
 *     descriptor. All supported idProduct's are define'd as
 *     MACUSB_PRODUCT_ID_TYPE. Appropriate types are define'd as
 *     MACUSB_TYPE_STR below,
 *   o N is bcdDevice in the device default descriptor (unsigned integer),
 *   o X is a unique number being set by kernel (unsigned integer).
 *
 * At any moment device node state is set to one of five states:
 *
 *   MACUSB_STATE_FREE            free, no IO in progress
 *   MACUSB_STATE_SYNC_WRITE      sync TX in progress
 *   MACUSB_STATE_SYNC_BUSY       sync TX done successfully
 *   MACUSB_STATE_SYNC_READ       sync RX in progress
 *   MACUSB_STATE_ASYNC_BUSY      executing request, completion flag loop is
 *                                running
 *   MACUSB_STATE_ASYNC_MAC_TOUT  executing request, completion flag loop has
 *                                finished, waiting for RX URB completion
 *                                handler
 *   MACUSB_STATE_ASYNC_URB_TOUT  RX URB completion handler timeout.
 *                                The state is changed to:
 *                                  * FREE by successful RX URB completion
 *                                    handler,
 *                                  * ERR by failed RX URB completion handler
 *                                    or by ioctl(MACUSB_IOC_RQ_STOP)
 *   MACUSB_STATE_CONF            configuration is in proggress
 *   MACUSB_STATE_ERR             failed to execute request
 *   MACUSB_STATE_END             end of device lifecycle
 *
 * State change is synchronized with spin lock to allow some operations
 * be run concurrently on SMP systems for the same device node.
 *
 * Example of asynchronous request:
 *
 *   const char *path = "/dev/macusb/macusb_le98_1-0";
 *   fd = open( path )
 *   if ( ioctl( fd, MACUSB_IOC_RQ_EOT, &cmd_buffer ) )
 *     perror( path );
 *   close( fd );
 *
 * Example of synchronous request:
 *
 *   const char *path = "/dev/macusb/macusb_le98_1-0";
 *   fd = open( path )
 *   // send commands
 *   if ( (bytes=write( fd, &out_buffer, count )) < 0 )
 *     perror( path );
 *   // receive answer
 *   if ( (bytes=read( fd, &in_buffer, in_buffer_size )) < 0 )
 *     perror( path );
 *   close( fd );
 *
 * NB: Atomicity of request execution is guaranteed, order of RX URBs is
 *     congruent to order of TX URBs for concurrent asynchronous AND
 *     synchronous requests always. The driver is thread-safe as of
 *     concurrent syscalls.
 *
 * NB2: The driver is NOT thread-safe as of order of concurrent requests.
 *     Threads executing requests on the same device concurrently must be
 *     synchronized in user space.
 *
 * NB3: If device state was changed to MACUSB_STATE_ERR the device should be
 *     at least cleared with MACUSB_IOC_CLEAR to erase possibly suspended
 *     bulk-out URB. Clearing is safe for all finished requests because
 *     completion handler doesn't change RX data of MAC'ed RX URBs.
 *     It is up to user to reset device or not somehow.
 *
 * NB4:  ioctl arg is of type uint64_t to make cmd name invariant by
 *       32/64-bit process context
 *
 * NB5: pointers in interface structures should have 64-bit width by the same
 *      reason
 */
#ifndef _MACUSB_H_
#define _MACUSB_H_ 1

#ifdef __cplusplus
extern "C" {
#endif

/* Device dependent */
#define MACUSB_VENDOR_ID               0x04b4

/* #define MACUSB_PRODUCT_ID              0x1002 default */
/* supported idProduct in device default descriptor */
#define MACUSB_PRODUCT_ID_LE99S        0x3301
#define MACUSB_PRODUCT_ID_LE98         0x3302
#define MACUSB_PRODUCT_ID_EM11         0x3303

/* driver-specific prefix */
#define MACUSB_NODE_PREFIX             "macusb"

/* device type string */
#define MACUSB_LE99S_STR               "le99s"
#define MACUSB_LE98_STR                "le98"
#define MACUSB_EM11_STR                "em11"

/* Get a minor range for your devices from the usb maintainer */
#define MACUSB_MINOR_BASE              192

#define MACUSB_IOC_MAGIC               'x'

#define MACUSB_VERSION_CODE            0x010202
#define MACUSB_VERSION_STR             "1.2.2"
#define MACUSB_VERSION(a,b,c)          (((a) << 16) + ((b) << 8) + (c))

/* Size of buffer to transmit data from.

   It is defined by size of the OUT endpoint buffer */
#define MACUSB_TXBUFSIZE               512

/* Size of buffer to receive data into.

   The RX buffer is vmalloc'ed (thanks to scatter-gather DMA in USB2
   controllers) so the size is not limited by continuous phys mem region (up
   to 128K only).

   It is defined by size of the IN endpoint buffer */
#define MACUSB_RXBUFSIZE               512

/* The parameters below could be set with
     MACUSB_IOC_SET_SYNC_CFG or
     MACUSB_IOC_SET_ASYNC_CFG
   for runtime. */

/* TX timeout in milliseconds in sync mode */
#define MACUSB_SYNC_TXTOUT             3000

/* RX timeout in milliseconds in sync mode */
#define MACUSB_SYNC_RXTOUT             3000

/* The number of reserved URBs in async mode. One should take into account:

   * Maximum number of bulk transactions for one USB microframe in accordance
     with USB2.0 spec for 8-byte payload is 119 (Table 5-10). USB microframes
     can be underused for a smaller value.

   * Peak system or extra USB loads can be significant so all URBs can be busy
     too frequently. The number should be increased in the case to soften
     effect. Histograms
       /sys/class/usb/macusbX-Y/device/bulk_out_urb_occupancy
       /sys/class/usb/macusbX-Y/device/bulk_in_urb_occupancy
     are provided to monitor URB occupancy. Big last values in the histos
     notify of URB starvation.
 */
#define MACUSB_ASYNC_URBNUM            120
#define MACUSB_ASYNC_MAX_URBNUM        (1<<10)

/* MAC timeout in microseconds in async mode. Should be changed only when
   system responsiveness is under question.

   NB: N16/N32 requests run two busy-wait loops, so real timeout is 2x mac_tout

   NB2: URB callback is run commonly at the next microframe border.
   If the timeout is to be increased for long-playing requests (e.g. MISS PChI
   command) it should be done with URB completion handler timeout
   else cpu core would be blocked too long so system responsiveness
   would suffer.

   NB3: Setting the timeout to zero disables MAC because the only check
   of completion flag in busy-wait loop in some nsec should fail with real
   USB device always
*/
#define MACUSB_ASYNC_MAX_MAC_TOUT      (125*2)
#define MACUSB_ASYNC_MAC_TOUT          MACUSB_ASYNC_MAX_MAC_TOUT

/* Timeout of URB completion handlers in milliseconds in async mode */
#define MACUSB_ASYNC_URB_TOUT          3000

/*****************************************************************************/
/* interface structures */

/* request buffer */
typedef struct macusb_buffer
{
  uint8_t   tx_buf[MACUSB_TXBUFSIZE]; /* buffer to transmit data from (TX) */
  uint32_t  tx_bytes;      /* number of bytes to send */
  uint8_t   rx_buf[MACUSB_RXBUFSIZE]; /* buffer to receive data into (RX) */
  uint32_t  rx_bytes;      /* number of received bytes */
  uint32_t  rx_eot_offset; /* offset of end-of-transfer mark in RX buffer.
			       Used by ioctl(MACUSB_IOC_RQ_EOT) */
  int32_t   rx_urbstatus;  /* RX URB status in completion handler */
  uint32_t  state;         /* device node state after ioctl command execution */
} __attribute__ ((__packed__)) macusb_buffer_t;

/* configuration parameters for synchronous transfers */
typedef struct macusb_sync_cfg
{
  uint32_t tx_tout;       /* TX timeout in milliseconds */
  uint32_t rx_tout;       /* RX timeout in milliseconds */
} __attribute__ ((__packed__)) macusb_sync_cfg_t;

/* configuration parameters for asynchronous transfers */
typedef struct macusb_async_cfg
{
  uint32_t urb_num;       /* number of reserved TX/RX URBs */
  uint32_t mac_tout;      /* MAC timeout in microseconds */
  uint32_t urb_tout;      /* timeout of URB completion handlers in milliseconds */
} __attribute__ ((__packed__)) macusb_async_cfg_t;

/* device node states */
typedef enum {
  MACUSB_STATE_FREE = 0,       /* Free, no IO in progress */
  MACUSB_STATE_SYNC_WRITE,     /* Sync TX in progress */
  MACUSB_STATE_SYNC_BUSY,      /* Sync TX done successfully */
  MACUSB_STATE_SYNC_READ,      /* Sync RX in progress */
  MACUSB_STATE_ASYNC_BUSY,     /* Executing request, completion flag
				  loop is running */
  MACUSB_STATE_ASYNC_MAC_TOUT, /* Executing request, completion flag
				  loop has finished, waiting for RX URB
				  completion handler */
  MACUSB_STATE_ASYNC_URB_TOUT, /* RX URB completion handler timeout */
  MACUSB_STATE_CONF,           /* configuration is in progress */
  MACUSB_STATE_ERR,            /* Failed to execute request */
  MACUSB_STATE_END,            /* End of device lifecycle */
} macusb_dev_state_t;

/* size of URB occupancy histogram */
#define MACUSB_URB_OCCUPANCY_HISTO_SIZE  10

/* pipe status info

   NB: updated for asynchronous requests only */
typedef struct macusb_pipe_status {
  uint64_t bytes;              /* overall number of transferred bytes */
  uint64_t mac_tout_urbs;      /* overall number of URBs with MAC timeout */
  uint64_t completed_urbs;     /* overall number of completed URBs */
  uint64_t size_mismatch_urbs; /* overall number of broken URBs (size
				   mismatch in completion handler) */
  uint64_t bad_status_urbs;    /* overall number of failed URBs (non-zero
				   status in completion handler) */
  uint64_t urb_occupancy[MACUSB_URB_OCCUPANCY_HISTO_SIZE]; /* URB occupancy histogram */
  uint32_t busy_urbs;          /* number of URBs in use */
} __attribute__ ((__packed__)) macusb_pipe_status_t;

/* device status info */
typedef struct macusb_status
{
  uint32_t                  state;     /* device node state */
  int32_t                   err;       /* number of last error */
  struct macusb_pipe_status bulk_out;  /* bulk-out status */
  struct macusb_pipe_status bulk_in;   /* bulk-in status */
} __attribute__ ((__packed__)) macusb_status_t;

/*****************************************************************************/
/* RX */

/* end-of-transfer mark, tail of 4 bytes length */
#define MACUSB_EOT_DATATYPE  uint32_t
#define MACUSB_EOT           0xFFFFFFFF

/*  RX rq header in variable length answer */
typedef struct macusb_rq_rx_prepacket {
  uint16_t           dummy1;      /* decoded by application layer */
  uint16_t           len    : 10; /* length of second packet */
  uint16_t           dummy2 : 6;  /* decoded by application layer */
  MACUSB_EOT_DATATYPE eot;         /* end-of-transfer mark */
} __attribute__ ((__packed__)) macusb_rq_rx_prepacket_t;

/* length of second packet payload
#define MACUSB_RQ_RX_LEN ( rq_first_rx_packet )	\
  ( rq_first_rx_packet.pld.len ) */

/*****************************************************************************/
/* ioctl commands */

/* get USB Device ID */
#define MACUSB_IOC_DEVICE_ID      _IO(MACUSB_IOC_MAGIC, 0)

/* get USB Product ID */
#define MACUSB_IOC_PRODUCT_ID     _IO(MACUSB_IOC_MAGIC, 1)

/* reset USB device, usb_reset_device(9):
      "Warns all drivers bound to registered interfaces (using their pre_reset
       method), performs the port reset, and then lets the drivers know that
       the reset is over (using their post_reset method)."

   NB: dangerous, kills all operations with device */
#define MACUSB_IOC_RESET_DEVICE   _IO(MACUSB_IOC_MAGIC, 2)

/* reset bus controller

   NB: dangerous, kills all operations with device */
#define MACUSB_IOC_RESET_CTL      _IO(MACUSB_IOC_MAGIC, 3)

/* Clear states, counters, buffers.

   NB: State should be MACUSB_STATE_FREE or MACUSB_STATE_ERR */
#define MACUSB_IOC_CLEAR          _IO(MACUSB_IOC_MAGIC, 4)

/* Get status of device node.
   status.state -- device state, see description of device node states for
     more info,
   status.err -- errno of last operation,
   status.bulk_out/in -- status for TX/RX pipes respectively. See description of
     'macusb_pipe_status_t' for more info.
   arg type: (macusb_status_t *) */
#define MACUSB_IOC_STATUS         _IOR(MACUSB_IOC_MAGIC, 5, uint64_t)

/* Stop synchronous/asynchronous request in progress.
   arg type: (macusb_status_t *) is set for success
   errno is set to EINTR if syscall was interrupted by a signal
   errno is set to EFAULT if syscall failed to copy state */
#define MACUSB_IOC_STOP_RQ        _IOR(MACUSB_IOC_MAGIC, 6, uint64_t)

/* Copy data buffer of the last synchronous/asynchronous request.
   arg type: (macusb_buffer_t *) */
#define MACUSB_IOC_COPY_RQ        _IOR(MACUSB_IOC_MAGIC, 7, uint64_t)

/*****************************************************************************/

/* Get sync transfer parameters
   arg type: (macusb_sync_cfg_t *) */
#define MACUSB_IOC_GET_SYNC_CFG   _IOR(MACUSB_IOC_MAGIC, 8, uint64_t)

/* Set sync transfer parameters. Size of transfer buffer should be:
     < MACUSB_BULK_IN_MAX_BUF_SIZE
     > wMaxPacketSize,
     size % wMaxPacketSize = 0
   arg type: (macusb_sync_cfg_t *) */
#define MACUSB_IOC_SET_SYNC_CFG   _IOW(MACUSB_IOC_MAGIC, 9, uint64_t)

/*****************************************************************************/

/* Get async transfer parameters
   arg type: (macusb_async_cfg_t *) */
#define MACUSB_IOC_GET_ASYNC_CFG  _IOR(MACUSB_IOC_MAGIC, 10, uint64_t)

/* Set async transfer parameters. Size of transfer buffer should be:
     < MACUSB_BULK_IN_MAX_BUF_SIZE
     > wMaxPacketSize,
     size % wMaxPacketSize = 0
     arg type: (macusb_async_cfg_t *) */
#define MACUSB_IOC_SET_ASYNC_CFG  _IOW(MACUSB_IOC_MAGIC, 11, uint64_t)

/* Execute asynchronous request with pointed length answer
   arg type: (macusb_buffer_t *) */
#define MACUSB_IOC_RQ_EOT         _IOWR(MACUSB_IOC_MAGIC, 12, uint64_t)

/* Execute asynchronous request with variable length answer: N 16-bit words (SNR)
   arg type: (macusb_buffer_t *) */
#define MACUSB_IOC_RQ_N16         _IOWR(MACUSB_IOC_MAGIC, 13, uint64_t)

/* Execute asynchronous request with variable length answer: N 32-bit words (SDR)
   arg type: (macusb_buffer_t *) */
#define MACUSB_IOC_RQ_N32         _IOWR(MACUSB_IOC_MAGIC, 14, uint64_t)

/* Execute asynchronous request with echo answer. Command is used to debug test
   firmware. Should not be used with full-fledged firmware.
   arg type: (macusb_buffer_t *)

   NB: Echo RX end-of-transfer mark is taken as the last byte of TX buffer */
#define MACUSB_IOC_RQ_ECHO        _IOWR(MACUSB_IOC_MAGIC, 15, uint64_t)

/*****************************************************************************/

#define MACUSB_IOC_MAXNR         16

#ifdef __cplusplus
}
#endif

#endif /* _MACUSB_H_ */
