#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <time.h>
#include <glob.h>
#include <endian.h>

#include "macusb.h"
#include "bcu/bcu_reg.h"
#include "bcu/bcu_le98.h"

/* Are O_DIRECT, O_SYNC, O_DSYNC to be used with devfs? */
#define BCU_LE98_OPEN_FLAGS  (O_RDWR)

/* location of LE98 device files */
#define BCU_LE98_DEV_DIR     "/dev"

/* hidden funcs */

#if defined __USE_MISC || defined __USE_XOPEN2K8
static int
is_older( struct timespec t0,
	  struct timespec t1 )
#else
struct mytime {
  time_t            tv_sec;
  unsigned long int tv_nsec;
};
static int
is_older( struct mytime t0,
	  struct mytime t1 )
#endif
{
  if ( t0.tv_sec < t1.tv_sec )
    return 1;
  if ( t0.tv_sec > t1.tv_sec )
    return 0;
  if ( t0.tv_nsec < t1.tv_nsec )
    return 1;
  if ( t0.tv_nsec > t1.tv_nsec )
    return 0;

  return 0;
}

/* NB: the function converts TX data into local buffer and converts RX data from local buffer in accordance with byte order of the particular machine (device is supposed to be 16-bit little endian) */
static int
_exec_buf( bcu_le98_t      *self,
	   macusb_buffer_t *ptr )
{
  int rv=0;
  macusb_status_t st;
  macusb_buffer_t buf=*ptr;
  int save_rv=0, save_errno=0;

#if __BYTE_ORDER != __LITTLE_ENDIAN
  unsigned int i=0;
  u_int16_t *orig=(u_int16_t *)ptr->tx_buf;
  u_int16_t *copy=(u_int16_t *)buf.tx_buf;
  for ( i=0; i<buf.tx_bytes/2; i++ )
    *copy++ = htole16( *orig++ );
#endif

  errno = 0;
  if ( !(self->transfer_flags & BCU_LE98_SYNC_MODE) ) {
    if ( (rv=ioctl( self->fd, MACUSB_IOC_RQ_EOT, &buf )) ) {
      save_rv = rv;
      save_errno = errno;
      /* URB timeout or interrupted, request should be stopped */
      if ( buf.state == MACUSB_STATE_ASYNC_MAC_TOUT ||
	   buf.state == MACUSB_STATE_ASYNC_URB_TOUT ) {
	if ( (rv=ioctl( self->fd, MACUSB_IOC_STOP_RQ, &st )) )
	  return rv;
	buf.state = st.state;
      }
      rv = save_rv;
      errno = save_errno;
    }

#if __BYTE_ORDER != __LITTLE_ENDIAN
    orig=(u_int16_t *)buf.rx_buf;
    copy=(u_int16_t *)ptr->rx_buf;
    for ( i=0; i<buf.tx_bytes/2; i++ )
      *copy++ = le16toh( *orig++ );
#else
    memcpy( ptr->rx_buf, buf.rx_buf, buf.rx_bytes );
#endif

  } else {
    if ( (rv=write( self->fd, buf.tx_buf, buf.tx_bytes )) >= 0 ) {
      if ( rv == buf.tx_bytes ) {
	if ( (rv=read( self->fd, buf.rx_buf, MACUSB_RXBUFSIZE )) >= 0 ) {
	  buf.rx_bytes = rv;

#if __BYTE_ORDER != __LITTLE_ENDIAN
	  orig=(u_int16_t *)buf.rx_buf;
	  copy=(u_int16_t *)ptr->rx_buf;
	  for ( i=0; i<buf.tx_bytes/2; i++ )
	    *copy++ = le16toh( *orig++ );
#else
	  memcpy( ptr->rx_buf, buf.rx_buf, buf.rx_bytes );
#endif

	  ptr->rx_bytes = buf.rx_bytes;
	  return 0;
	}
      } else {
	rv = -1;
      }
    }
    /* in any case request should be stopped */
    if ( !ioctl( self->fd, MACUSB_IOC_STOP_RQ, &st ) ) {
      buf.state = st.state;
    }
    if ( !errno )
      errno = EIO;
  }
  ptr->rx_bytes = buf.rx_bytes;
  ptr->rx_urbstatus = buf.rx_urbstatus;
  ptr->state = buf.state;
  return rv;
}

static int
_encode_cmd( macusb_buffer_t *buf,
	     bcu_le98_cmd_t   cmd,
	     u_int16_t        addr,
	     u_int16_t        data )
{
  int rv=0;
  switch ( cmd ) {
  case BCU_LE98_INIT_CTL_CMD:
    BCU_ENCODE_INIT_CTL( *buf, rv );
    break;
  case BCU_LE98_MODE_GET_CMD:
    BCU_ENCODE_READ_REG( *buf, rv, BCU_LE98_MISS_MODE_ADDR );
    break;
  case BCU_LE98_MODE_SET_CMD:
    BCU_ENCODE_WRITE_REG( *buf, rv, BCU_LE98_MISS_MODE_ADDR, data );
    break;
  case BCU_LE98_REG_READ_CMD:
    BCU_ENCODE_READ_REG( *buf, rv, addr );
    break;
  case BCU_LE98_REG_WRITE_CMD:
    BCU_ENCODE_WRITE_REG( *buf, rv, addr, data );
    break;
  case BCU_LE98_NAF_READ_CMD:
    BCU_ENCODE_WRITE_REG( *buf, rv, BCU_LE98_CMD_ADDR, addr );
    if ( !rv ) {
      BCU_ENCODE_READ_REG( *buf, rv, BCU_LE98_DATA_ADDR );
      if ( rv ) {
	buf->tx_bytes -= sizeof(bcu_tx_packet_t);
      }
    }
    break;
  case BCU_LE98_NAF_WRITE_CMD:
    BCU_ENCODE_WRITE_REG( *buf, rv, BCU_LE98_DATA_ADDR, data );
    if ( !rv ) {
      BCU_ENCODE_WRITE_REG( *buf, rv, BCU_LE98_CMD_ADDR, addr );
      if ( rv ) {
	buf->tx_bytes -= sizeof(bcu_tx_packet_t);
      }
    }
    break;
  case BCU_LE98_NAF_CONTROL_CMD:
    BCU_ENCODE_WRITE_REG( *buf, rv, BCU_LE98_CMD_ADDR, addr );
    break;
  case BCU_LE98_SNR_START_CMD:
    if ( buf->tx_bytes == 0 ) {
      BCU_ENCODE_WRITE_REG( *buf, rv, BCU_LE98_MISS_MODE_ADDR, BCU_LE98_MISS_SNR_MODE );
      if ( !rv ) {
	BCU_ENCODE_READ_REG( *buf, rv, BCU_LE98_STATUS_ADDR );
	if ( rv ) {
	  buf->tx_bytes -= sizeof(bcu_tx_packet_t);
	}
      }
    } else {
      rv = BCU_ENCODE_BUF_NOT_EMPTY_ERR;
    }
    break;
  case BCU_LE98_SNR_READ_CMD:
    if ( buf->tx_bytes == 0 ) {
      BCU_ENCODE_READ_REG( *buf, rv, BCU_LE98_CMD_ADDR );
      if ( !rv ) {
	BCU_ENCODE_READ_REG( *buf, rv, BCU_LE98_STATUS_ADDR );
	if ( rv ) {
	  buf->tx_bytes -= sizeof(bcu_tx_packet_t);
	}
      }
    } else {
      rv = BCU_ENCODE_BUF_NOT_EMPTY_ERR;
    }
    break;
  case BCU_LE98_SNR_STOP_CMD:
    BCU_ENCODE_WRITE_REG( *buf, rv, BCU_LE98_MISS_MODE_ADDR, BCU_LE98_MISS_ADDR_MODE );
    break;
  case BCU_LE98_SDR_START_CMD:
    if ( buf->tx_bytes == 0 ) {
      BCU_ENCODE_WRITE_REG( *buf, rv, BCU_LE98_MISS_MODE_ADDR, BCU_LE98_MISS_SDR_MODE );
      if ( !rv ) {
	BCU_ENCODE_READ_REG( *buf, rv, BCU_LE98_STATUS_ADDR );
	if ( rv ) {
	  buf->tx_bytes -= sizeof(bcu_tx_packet_t);
	}
      }
    } else {
      rv = BCU_ENCODE_BUF_NOT_EMPTY_ERR;
    }
    break;
  case BCU_LE98_SDR_READ_CMD:
    if ( buf->tx_bytes == 0 ) {
      BCU_ENCODE_READ_REG( *buf, rv, BCU_LE98_DATA_ADDR );
      if ( !rv ) {
	BCU_ENCODE_READ_REG( *buf, rv, BCU_LE98_CMD_ADDR );
	if ( rv ) {
	  buf->tx_bytes -= sizeof(bcu_tx_packet_t);
	}
	if ( !rv ) {
	  BCU_ENCODE_READ_REG( *buf, rv, BCU_LE98_STATUS_ADDR );
	  if ( rv ) {
	    buf->tx_bytes -= 2 * sizeof(bcu_tx_packet_t);
	  }
	}
      }
    } else {
      rv = BCU_ENCODE_BUF_NOT_EMPTY_ERR;
    }
    break;
  case BCU_LE98_SDR_STOP_CMD:
    BCU_ENCODE_WRITE_REG( *buf, rv, BCU_LE98_MISS_MODE_ADDR, BCU_LE98_MISS_ADDR_MODE );
    break;
  default:
    rv = BCU_ENCODE_NO_SUCH_COMMAND_ERR;
  }
  return rv;
}

static int
_decode_cmd( macusb_buffer_t *buf,
	     unsigned int    *offset,
	     bcu_le98_cmd_t   cmd,
	     u_int16_t       *state,
	     u_int16_t       *addr,
	     u_int16_t       *data )
{
  int rv=0;
  /* turn off for variable length answer */
  if ( buf->tx_bytes != buf->rx_bytes - sizeof( MACUSB_EOT_DATATYPE ) )
    return BCU_DECODE_DATA_LEN_ERR;
  if ( *((MACUSB_EOT_DATATYPE *)(buf->rx_buf + buf->rx_bytes - sizeof(MACUSB_EOT_DATATYPE))) != MACUSB_EOT )
    return BCU_DECODE_NO_EOT_ERR;
  switch ( cmd ) {
  case BCU_LE98_INIT_CTL_CMD:
    BCU_DECODE_INIT_CTL( *buf, *offset, rv, *data );
    break;
  case BCU_LE98_MODE_GET_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    break;
  case BCU_LE98_MODE_SET_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    break;
  case BCU_LE98_REG_READ_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    break;
  case BCU_LE98_REG_WRITE_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    break;
  case BCU_LE98_NAF_READ_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *addr );
    if ( !rv ) {
      BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    }
    break;
  case BCU_LE98_NAF_WRITE_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    if ( !rv ) {
      BCU_DECODE_REG_OP( *buf, *offset, rv, *addr );
    }
    break;
  case BCU_LE98_NAF_CONTROL_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *addr );
    break;
  case BCU_LE98_SNR_START_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    if ( !rv ) {
      BCU_DECODE_REG_OP( *buf, *offset, rv, *state );
    }
    break;
  case BCU_LE98_SNR_READ_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *addr );
    if ( !rv ) {
      BCU_DECODE_REG_OP( *buf, *offset, rv, *state );
    }
    break;
  case BCU_LE98_SNR_STOP_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    break;
  case BCU_LE98_SDR_START_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    if ( !rv ) {
      BCU_DECODE_REG_OP( *buf, *offset, rv, *state );
    }
    break;
  case BCU_LE98_SDR_READ_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    if ( !rv ) {
      BCU_DECODE_REG_OP( *buf, *offset, rv, *addr );
      if ( !rv ) {
	BCU_DECODE_REG_OP( *buf, *offset, rv, *state );
      }
    }
    break;
  case BCU_LE98_SDR_STOP_CMD:
    BCU_DECODE_REG_OP( *buf, *offset, rv, *data );
    break;
  default:
    rv = BCU_DECODE_NO_SUCH_COMMAND_ERR;
  }
  return rv;
}

static int
_reset( bcu_le98_t *self,
	int         reset_rq )
{
  int rv=0;
  int save_errno=0;
  rv = ioctl( self->fd, reset_rq );
  save_errno = errno;
  bcu_le98_close( self );
  errno = save_errno;
  return rv;
}

/*****************************************************************************/
/* Interface */

void
bcu_le98_init( bcu_le98_t *self )
{
  memset( self, 0, sizeof( *self ) );
  self->id = -1;
  self->dev_dir = BCU_LE98_DEV_DIR;
  self->fd = -1;
  self->transfer_flags = BCU_LE98_TANSFER_FLAGS;
}

int
bcu_le98_fini( bcu_le98_t *self )
{
  return bcu_le98_close( self );
}

int
bcu_le98_open_id( bcu_le98_t   *self,
		  unsigned int  id )
{
  int rv=0;
  int i, idx=-1;
  char pattern[BCU_LE98_PATH_BUFSIZE];
  char path[BCU_LE98_PATH_BUFSIZE];
  glob_t globbuf;
  struct stat statbuf;
#if defined __USE_MISC || defined __USE_XOPEN2K8
  struct timespec t0, t1;
#else
  struct mytime t0, t1;
#endif
  errno=0;

  memset( &globbuf, 0, sizeof( globbuf ) );
  /* wildcard pattern 'macusbID-*' matches all LE98 devices with ID */
  snprintf( pattern, BCU_LE98_PATH_BUFSIZE, "%s/"MACUSB_NODE_PREFIX"%u-*", self->dev_dir, id );

  if ( (rv=glob( pattern, GLOB_NOSORT, NULL, &globbuf )) )
    return rv;

  /* macusb_delete() in driver macusb can be run after LE98 has been
     reenumerated. Search for file with latest status change time. */
  for ( i=0; i<globbuf.gl_pathc; i++ ) {
    if ( stat( globbuf.gl_pathv[i], &statbuf ) )
      continue;
#if defined __USE_MISC || defined __USE_XOPEN2K8
    t1 = statbuf.st_ctim;
#else
    t1.tv_sec = statbuf.st_ctime;
    t1.tv_nsec = statbuf.st_ctimensec;
#endif
    if ( !i || is_older( t0, t1 ) > 0 ) {
      t0 = t1;
      idx = i;
    }
  }
  if ( idx >= 0 ) {
    snprintf( path, BCU_LE98_PATH_BUFSIZE, "%s", globbuf.gl_pathv[idx] );
    if ( !(rv=bcu_le98_open_path( self, path )) )
      self->id = id;

  } else {
    rv = ENODEV;
  }
  globfree( &globbuf );
  return rv;
}

int
bcu_le98_open_path( bcu_le98_t *self,
		    const char *path )
{
  int rv=0;
  bcu_le98_close( self );
  if ( !path || !strlen(path) || strlen(path) >= BCU_LE98_PATH_BUFSIZE )
    return 1;
  self->id = -1;
  strcpy( self->path, path );
  if ( (self->fd=open( self->path, BCU_LE98_OPEN_FLAGS )) < 0 )
    return -1;
  return rv;
}

int
bcu_le98_close( bcu_le98_t *self )
{
  int rv=0;
  if ( self->fd >= 0 ) {
    rv = close( self->fd );
    self->fd = -1;
  }
  return rv;
}

int
bcu_le98_status( bcu_le98_t      *self,
		 macusb_status_t *st )
{
  return ioctl( self->fd, MACUSB_IOC_STATUS, st );
}

int
bcu_le98_reset( bcu_le98_t *self )
{
  return _reset( self, MACUSB_IOC_RESET_DEVICE );
}

int
bcu_le98_reset_ctl( bcu_le98_t *self )
{
  return _reset( self, MACUSB_IOC_RESET_CTL );
}

int
bcu_le98_clear( bcu_le98_t *self )
{
  return ioctl( self->fd, MACUSB_IOC_CLEAR );
}

int
bcu_le98_get_sync_cfg( bcu_le98_t        *self,
		       macusb_sync_cfg_t *cfg )
{
  return ioctl( self->fd, MACUSB_IOC_GET_SYNC_CFG, cfg );
}

int
bcu_le98_set_sync_cfg( bcu_le98_t        *self,
		       macusb_sync_cfg_t *cfg )
{
  return ioctl( self->fd, MACUSB_IOC_SET_SYNC_CFG, cfg );
}

int
bcu_le98_get_async_cfg( bcu_le98_t         *self,
			macusb_async_cfg_t *cfg )
{
  return ioctl( self->fd, MACUSB_IOC_GET_ASYNC_CFG, cfg );
}

int
bcu_le98_set_async_cfg( bcu_le98_t         *self,
			macusb_async_cfg_t *cfg )
{
  return ioctl( self->fd, MACUSB_IOC_SET_ASYNC_CFG, cfg );
}

int
bcu_le98_exec_cmd( bcu_le98_t     *self,
		   bcu_le98_cmd_t  cmd,
		   u_int16_t      *state,
		   u_int16_t      *addr,
		   u_int16_t      *data )
{
  int rv=0;
  macusb_buffer_t buf;
  unsigned int offset=0;
  memset( &buf, 0, sizeof( buf ) );
  if ( (rv=_encode_cmd( &buf, cmd, *addr, *data )) )
    return rv;
  if ( (rv=_exec_buf( self, &buf )) )
    return rv;
  return _decode_cmd( &buf, &offset, cmd, state, addr, data );
}

int
bcu_le98_exec_buf( bcu_le98_t      *self,
		   macusb_buffer_t *buf )
{
  int rv = 0;
  if ( (rv=_exec_buf( self, buf )) )
    return rv;
  return 0;
}

int
bcu_le98_encode_cmd( macusb_buffer_t *buf,
		     bcu_le98_cmd_t   cmd,
		     u_int16_t        attr,
		     u_int16_t        data )
{
  return _encode_cmd( buf, cmd, attr, data );
}

int
bcu_le98_decode_cmd( macusb_buffer_t *buf,
		     unsigned int    *offset,
		     bcu_le98_cmd_t   cmd,
		     u_int16_t       *state,
		     u_int16_t       *addr,
		     u_int16_t       *data )
{
  return _decode_cmd( buf, offset, cmd, state, addr, data );
}

const char *
bcu_le98_state_str( unsigned int state )
{
  const char *statestr[MACUSB_STATE_END+1] = {
    "Free, no IO in progress",
    "Sync TX in progress",
    "Sync TX done successfully",
    "Sync RX in progress",
    "Executing request, completion flag loop is running",
    "Executing request, completion flag loop has finished, waiting for RX URB completion handler",
    "Request timeout",
    "Device is being configured",
    "Failed to execute request",
    "Device is being deleted"
  };
  return state < MACUSB_STATE_END+1 ? statestr[state] : NULL;
}
