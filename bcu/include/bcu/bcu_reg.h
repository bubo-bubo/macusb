/*
 * Bus controller with USB interface (BCU)
 *
 * Copyright (C) 2014 Alexey Filin
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * The header file defines register layer of BCU protocol stack.
 * The layer implements elemental operations with bus controller registers:
 *
 *   * errors reported by controller,
 *   * TX/RX packets for operations with registers,
 *   * types of operations with registers,
 *   * macros to en-/de-code register operations,
 *
 *
 */
#ifndef _BCU_REG_H_
#define _BCU_REG_H_ 1

#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* en-/decoding errors */
typedef enum bcu_error {
/* errors reported by bus controller */
  BCU_CTL_OK            = 0x0,    /* successful register operation */
  BCU_CTL_INIT_OK       = 0x10,   /* successful controller initialisation */
  BCU_CTL_LEN_ERR       = 0x20,   /* wrong length of RX packet */
  BCU_CTL_REG_ADDR_ERR  = 0x40,   /* wrong register address */
  BCU_CTL_BUS_TOUT_ERR  = 0x80,   /* bus timeout */

/* encoding errors */
  BCU_ENCODE_BUF_OVERRUN_ERR     = 0x100, /* TX buffer overrun */
  BCU_ENCODE_BUF_NOT_EMPTY_ERR   = 0x200, /* TX buffer not empty */
  BCU_ENCODE_NO_SUCH_COMMAND_ERR = 0x400, /* no such command */

/* decoding errors */
  BCU_DECODE_DATA_LEN_ERR        = 0x10000,  /* wrong RX data length */
  BCU_DECODE_FORMAT_ERR          = 0x20000,  /* format error */
  BCU_DECODE_NO_SUCH_COMMAND_ERR = 0x40000,  /* no such command */
  BCU_DECODE_NO_EOT_ERR          = 0x80000,  /* no end-of-transfer mark */
} bcu_error_t;

/* mask of bus controller errors */
#define BCU_CTL_ERROR_MASK     0xFF

/* mask of encoding errors */
#define BCU_ENCODE_ERROR_MASK  (0xFF00)

/* mask of decoding errors */
#define BCU_DECODE_ERROR_MASK  (0xFF0000)

/*****************************************************************************/
/* TX */

/* operations with bus controller registers */

/* read from register */
#define BCU_REG_READ  0x0

/* write to register */
#define BCU_REG_WRITE 0x1

/* initialize controller */
#define BCU_INIT_CTL  0xFF

/* TX header holds code of register operation */
typedef struct bcu_tx_header {
  u_int16_t reg_addr : 5; /* address of controller register */
  u_int16_t reg_op1  : 1; /* reserved */
  u_int16_t reg_op2  : 1; /* reserved */
  u_int16_t reg_op3  : 1; /* type of operation with controller register */
  u_int16_t init_ctl : 8; /* controller initialisation bits */
} __attribute__ ((__packed__)) bcu_tx_header_t;

/* convenience macros to fill TX packet header */
#define BCU_FILL_TX_HDR( _hdr, _address, _operation )			\
  do {									\
    (_hdr).reg_addr = (_address);					\
    (_hdr).reg_op1  = 0;						\
    (_hdr).reg_op2  = 0;						\
    (_hdr).reg_op3  = (_operation);					\
    (_hdr).init_ctl = 0;						\
  } while ( 0 )

/* TX packet contains TX header and payload */
typedef struct bcu_tx_packet {
  struct bcu_tx_header hdr;
  u_int16_t            pld;
} __attribute__ ((__packed__)) bcu_tx_packet_t;

/* Macros to encode register operations into packets

   NB: All encoding macros don't change data size in TX buffer if failed */

/* fill TX packet for operation 'initialise controller' */
#define BCU_FILL_INIT_CTL_TX_PACKET( _packet )				\
  do {									\
    (_packet).hdr.reg_addr = 0;						\
    (_packet).hdr.reg_op1  = 0;						\
    (_packet).hdr.reg_op2  = 0;						\
    (_packet).hdr.reg_op3  = 0;						\
    (_packet).hdr.init_ctl = BCU_INIT_CTL;				\
    (_packet).pld = 0;							\
  } while ( 0 )

/* fill TX packet for operation 'read register' */
#define BCU_FILL_READ_REG_TX_PACKET( _packet, _address )		\
  do {									\
    BCU_FILL_TX_HDR( (_packet).hdr, (_address), BCU_REG_READ );		\
    (_packet).pld = 0;							\
  } while ( 0 )

/* fill TX packet for operation 'write to register' */
#define BCU_FILL_WRITE_REG_TX_PACKET( _packet, _address, _data )	\
  do {									\
    BCU_FILL_TX_HDR( (_packet).hdr, (_address), BCU_REG_WRITE );	\
    (_packet).pld = (_data);						\
  } while ( 0 )


/* Macros to encode register operations into 'struct macusb_buffer_t' */

/* encode 'READ register' operation
 mbuf  user buffer
 errors  encoding errors
 address  register address to read from */
#define BCU_ENCODE_READ_REG( _mbuf, _errors, _reg_address )		\
  do {									\
    (_errors) = 0;							\
    if ( (_mbuf).tx_bytes + sizeof(bcu_tx_packet_t) <= MACUSB_TXBUFSIZE ) { \
      BCU_FILL_READ_REG_TX_PACKET( *((bcu_tx_packet_t *)((_mbuf).tx_buf + (_mbuf).tx_bytes)), (_reg_address) ); \
      (_mbuf).tx_bytes += sizeof(bcu_tx_packet_t);			\
      (_mbuf).rx_eot_offset += sizeof(bcu_rx_packet_t);			\
    } else {								\
      (_errors) = BCU_ENCODE_BUF_OVERRUN_ERR;				\
    }									\
  } while ( 0 )

/* encode 'WRITE to register' operation
 mbuf  user buffer
 errors  encoding errors
 address  register address to write
 data  data to write */
#define BCU_ENCODE_WRITE_REG( _mbuf, _errors, _reg_address, _data )	\
  do {									\
    (_errors) = 0;							\
    if ( (_mbuf).tx_bytes + sizeof(bcu_tx_packet_t) <= MACUSB_TXBUFSIZE ) { \
      BCU_FILL_WRITE_REG_TX_PACKET( *((bcu_tx_packet_t *)((_mbuf).tx_buf + (_mbuf).tx_bytes)), (_reg_address), (_data) ); \
      (_mbuf).tx_bytes += sizeof(bcu_tx_packet_t);			\
      (_mbuf).rx_eot_offset += sizeof(bcu_rx_packet_t);			\
    } else {								\
      (_errors) = BCU_ENCODE_BUF_OVERRUN_ERR;				\
    }									\
  } while ( 0 )

/* encode 'init controller' command
   mbuf  user buffer
   errors  encoding errors

   NB: 'init controller' command is defined on register layer cause it is
   defined in TX packet format */
#define BCU_ENCODE_INIT_CTL( _mbuf, _errors )				\
  do {									\
    if ( (_mbuf).tx_bytes + sizeof(bcu_tx_packet_t) <= MACUSB_TXBUFSIZE ) { \
      BCU_FILL_INIT_CTL_TX_PACKET( *((bcu_tx_packet_t *)((_mbuf).tx_buf + (_mbuf).tx_bytes)) ); \
      (_mbuf).tx_bytes += sizeof(bcu_tx_packet_t);			\
    } else {								\
      (_errors) = BCU_ENCODE_BUF_OVERRUN_ERR;				\
    }									\
  } while ( 0 )

/*****************************************************************************/
/* RX */

/* RX header */
typedef struct bcu_rx_header {
  u_int16_t reg_addr : 5; /* address of controller register */
  u_int16_t reg_op1  : 1; /* reserved */
  u_int16_t reg_op2  : 1; /* reserved */
  u_int16_t reg_op3  : 1; /* type of operation with controller register */
  u_int16_t errors   : 8; /* register operation errors */
} __attribute__ ((__packed__)) bcu_rx_header_t;

/* RX packet */
typedef struct bcu_rx_packet {
  struct bcu_rx_header hdr;
  u_int16_t            pld; /* payload */
} __attribute__ ((__packed__)) bcu_rx_packet_t;

/* RX prepacket in variable length answer

   NB: Unfortunately hw designer mixed request and command layers. Proper
       design should isolate implementation details of different layers from
       each other. The packet should match struct macusb_rq_rx_prepacket_t
       from macusb.h */
typedef struct bcu_rx_prepacket {
  u_int16_t ret_code;    /* command return code */
  u_int16_t len   : 10;  /* length of second packet */
  u_int16_t dummy : 6;   /* decoded by application layer */
  u_int32_t eot;         /* end-of-transfer mark */
} __attribute__ ((__packed__)) bcu_rx_prepacket_t;

/* Macro to decode controller register operation

   NB: Offset is incremented to next data or points to packet with decoding
   error */
#define BCU_DECODE_REG_OP( _mbuf, _offset, _errors, _data )		\
  do {									\
    if ( (_offset) + sizeof(bcu_tx_packet_t) <= (_mbuf).tx_bytes &&	\
	 (_offset) + sizeof(bcu_rx_packet_t) <= (_mbuf).rx_bytes) {	\
      if ( ((bcu_tx_packet_t *)((_mbuf).tx_buf+(_offset)))->hdr.reg_addr == ((bcu_rx_packet_t *)((_mbuf).rx_buf+(_offset)))->hdr.reg_addr && \
	   ((bcu_tx_packet_t *)((_mbuf).tx_buf+(_offset)))->hdr.reg_op1 == ((bcu_rx_packet_t *)((_mbuf).rx_buf+(_offset)))->hdr.reg_op1 && \
	   ((bcu_tx_packet_t *)((_mbuf).tx_buf+(_offset)))->hdr.reg_op2 == ((bcu_rx_packet_t *)((_mbuf).rx_buf+(_offset)))->hdr.reg_op2 && \
	   ((bcu_tx_packet_t *)((_mbuf).tx_buf+(_offset)))->hdr.reg_op3 == ((bcu_rx_packet_t *)((_mbuf).rx_buf+(_offset)))->hdr.reg_op3 ) { \
	if ( ((bcu_tx_packet_t *)((_mbuf).tx_buf+(_offset)))->hdr.init_ctl == BCU_INIT_CTL ) { \
	  if ( ((bcu_rx_packet_t *)((_mbuf).rx_buf+(_offset)))->hdr.errors == BCU_CTL_INIT_OK && \
	       ((bcu_rx_packet_t *)((_mbuf).rx_buf+(_offset)))->pld == 0 ) { \
	    (_errors) &= ~BCU_CTL_INIT_OK;				\
	  } else {							\
	    (_errors) |= ((bcu_rx_packet_t *)((_mbuf).rx_buf+(_offset)))->hdr.errors; \
	  }								\
	  (_data) = ((bcu_rx_packet_t *)((_mbuf).rx_buf+(_offset)))->pld; \
	} else if ( ( ((bcu_tx_packet_t *)((_mbuf).tx_buf+(_offset)))->hdr.reg_op3 == BCU_REG_WRITE && \
		      ((bcu_tx_packet_t *)((_mbuf).tx_buf+(_offset)))->pld == ((bcu_rx_packet_t *)((_mbuf).rx_buf+(_offset)))->pld ) || \
		    ((bcu_tx_packet_t *)((_mbuf).tx_buf+(_offset)))->hdr.reg_op3 == BCU_REG_READ ) { \
	  (_errors) |= ((bcu_rx_packet_t *)((_mbuf).rx_buf+(_offset)))->hdr.errors; \
	  (_data) = ((bcu_rx_packet_t *)((_mbuf).rx_buf+(_offset)))->pld; \
	  (_offset) += sizeof(bcu_rx_packet_t);				\
	} else {							\
	  (_errors) |= BCU_DECODE_FORMAT_ERR;				\
	}								\
      } else {								\
	(_errors) |= BCU_DECODE_FORMAT_ERR;				\
      }									\
    } else {								\
      (_errors) |= BCU_DECODE_DATA_LEN_ERR;				\
    }									\
  } while ( 0 )

/* decode 'init controller' command */
#define BCU_DECODE_INIT_CTL( _mbuf, _offset, _errors, _cmd )		\
  do {									\
    BCU_DECODE_REG_OP( (_mbuf), (_offset), (_errors), (_cmd) );		\
  } while ( 0 )


#ifdef __cplusplus
}
#endif

#endif /* _BCU_REG_H_ */
