#include "bcu/bcu_em11.h"
#include "bcu/bcu_dev.h"
#include "bcu/bcu_reg.h"
#include "macusb.h"

static int
_encode_cmd( void            *self,
	     macusb_buffer_t *buf,
	     unsigned int     cmd,
	     uint16_t         addr,
	     uint16_t         data )
{
  int rv=0;
  switch ( cmd ) {
  case BCU_DEV_CMD_INIT_CTL:
    BCU_ENCODE_INIT_CTL( *buf, rv );
    break;
  case BCU_DEV_CMD_MODE_GET:
    BCU_ENCODE_READ_REG( *buf, rv, BCU_EM11_ADDR_EMISS_MODE );
    break;
  case BCU_DEV_CMD_MODE_SET:
    BCU_ENCODE_WRITE_REG( *buf, rv, BCU_EM11_ADDR_EMISS_MODE, data );
    break;
  case BCU_DEV_CMD_REG_READ:
    BCU_ENCODE_READ_REG( *buf, rv, addr );
    break;
  case BCU_DEV_CMD_REG_WRITE:
    BCU_ENCODE_WRITE_REG( *buf, rv, addr, data );
    break;
  case BCU_DEV_CMD_NAF_READ:
    BCU_ENCODE_WRITE_REG( *buf, rv, BCU_EM11_ADDR_READ_CMD, addr );
    if ( !rv ) {
      BCU_ENCODE_READ_REG( *buf, rv, BCU_EM11_ADDR_DATA );
      if ( rv ) {
	buf->tx_bytes -= sizeof(bcu_tx_packet_t);
      }
    }
    break;
  case BCU_DEV_CMD_NAF_WRITE:
    BCU_ENCODE_WRITE_REG( *buf, rv, BCU_EM11_ADDR_DATA, data );
    if ( !rv ) {
      BCU_ENCODE_WRITE_REG( *buf, rv, BCU_EM11_ADDR_WRITE_CMD, addr );
      if ( rv ) {
	buf->tx_bytes -= sizeof(bcu_tx_packet_t);
      }
    }
    break;
  case BCU_DEV_CMD_NAF_CONTROL:
    BCU_ENCODE_WRITE_REG( *buf, rv, BCU_EM11_ADDR_WRITE_CMD, addr );
    break;
  case BCU_DEV_CMD_SNR_START:
    if ( buf->tx_bytes == 0 ) {
      BCU_ENCODE_WRITE_REG( *buf, rv, BCU_EM11_ADDR_EMISS_MODE, BCU_EM11_MODE_SNR );
    } else {
      rv = BCU_ENC_ERR_BUF_NOT_EMPTY;
    }
    break;
  case BCU_DEV_CMD_SNR_READ:
    BCU_ENCODE_READ_REG( *buf, rv, BCU_EM11_ADDR_SSON_AB );
    break;
  case BCU_DEV_CMD_SDR_START:
    if ( buf->tx_bytes == 0 ) {
      BCU_ENCODE_WRITE_REG( *buf, rv, BCU_EM11_ADDR_EMISS_MODE, BCU_EM11_MODE_SDR );
    } else {
      rv = BCU_ENC_ERR_BUF_NOT_EMPTY;
    }
    break;
  case BCU_DEV_CMD_SDR_READ:
    BCU_ENCODE_READ_REG( *buf, rv, BCU_EM11_ADDR_SSON_AB );
    BCU_ENCODE_READ_REG( *buf, rv, BCU_EM11_ADDR_SSOFF_DB );
    break;
  case BCU_DEV_CMD_SR_STATUS:
    if ( buf->tx_bytes == 0 ) {
      BCU_ENCODE_READ_REG( *buf, rv, BCU_EM11_ADDR_SR_STATUS );
    } else {
      rv = BCU_ENC_ERR_BUF_NOT_EMPTY;
    }
    break;
  default:
    rv = BCU_ENC_ERR_NO_SUCH_COMMAND;
  }
  return rv;
}

static int
_decode_cmd( void            *self,
	     macusb_buffer_t *buf,
	     unsigned int    *offset,
	     unsigned int     cmd,
	     uint16_t        *state,
	     uint16_t        *addr,
	     uint16_t        *data )
{
  int rv=0;
  /* turn off for variable length answer */
  if ( buf->tx_bytes != buf->rx_bytes - sizeof( MACUSB_EOT_DATATYPE ) )
    return BCU_DEC_ERR_DATA_LEN;
  if ( *((MACUSB_EOT_DATATYPE *)(buf->rx_buf + buf->rx_bytes - sizeof(MACUSB_EOT_DATATYPE))) != MACUSB_EOT )
    return BCU_DEC_ERR_NO_EOT;
  switch ( cmd ) {
  case BCU_DEV_CMD_INIT_CTL:
    BCU_DECODE_INIT_CTL( *buf, *offset, rv, *data );
    break;
  case BCU_DEV_CMD_MODE_GET:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    break;
  case BCU_DEV_CMD_MODE_SET:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    break;
  case BCU_DEV_CMD_REG_READ:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    break;
  case BCU_DEV_CMD_REG_WRITE:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    break;
  case BCU_DEV_CMD_NAF_READ:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *addr );
    if ( !rv ) {
      BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    }
    break;
  case BCU_DEV_CMD_NAF_WRITE:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    if ( !rv ) {
      BCU_DECODE_REG_OP( *buf, *offset, rv, *addr );
    }
    break;
  case BCU_DEV_CMD_NAF_CONTROL:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *addr );
    break;
  case BCU_DEV_CMD_SNR_START:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    break;
  case BCU_DEV_CMD_SNR_READ:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *addr );
    break;
  case BCU_DEV_CMD_SDR_START:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    break;
  case BCU_DEV_CMD_SDR_READ:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *addr );
    if ( !rv ) {
      BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    }
    break;
  case BCU_DEV_CMD_SR_STATUS:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *state );
    break;
  default:
    rv = BCU_DEC_ERR_NO_SUCH_COMMAND;
  }
  return rv;
}

/*****************************************************************************/
/* Interface */

_bcu_dev_vt_t *
bcu_em11_get_vt( void )
{
  static _bcu_dev_vt_t vt;
  static int is_vt_inited = 0; /* to prevent vt reinitialization during subsequent calls */
  if ( ! is_vt_inited ) {
    vt.encode_cmd = _encode_cmd;
    vt.decode_cmd = _decode_cmd;
    vt.name       = "bcu_em11";
    is_vt_inited  = 1;
  }
  return &vt;
}

int
bcu_em11_init( bcu_dev_t *self )
{
  int rv = bcu_dev_init( self );
  self->_vt = *bcu_em11_get_vt();
  return rv;
}
