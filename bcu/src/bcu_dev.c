#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <time.h>
#include <glob.h>
#include <endian.h>

#include "bcu/bcu_reg.h"
#include "bcu/bcu_dev.h"
#include "macusb.h"

/* Are O_DIRECT, O_SYNC, O_DSYNC to be used with devfs? */
#define BCU_DEV_OPEN_FLAGS  (O_RDWR)

/* location of DEV device files */
#define BCU_DEV_DEV_DIR     "/dev"

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
_exec_buf( bcu_dev_t       *self,
	   macusb_buffer_t *ptr )
{
  int rv=0;
  macusb_status_t st;
  macusb_buffer_t buf=*ptr;
  int save_rv=0, save_errno=0;

#if __BYTE_ORDER != __LITTLE_ENDIAN
  unsigned int i=0;
  uint16_t *orig=(uint16_t *)ptr->tx_buf;
  uint16_t *copy=(uint16_t *)buf.tx_buf;
  for ( i=0; i<buf.tx_bytes/2; i++ )
    *copy++ = htole16( *orig++ );
#endif

  errno = 0;
  if ( !(self->transfer_flags & BCU_DEV_FLAG_SYNC) ) {
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
    orig=(uint16_t *)buf.rx_buf;
    copy=(uint16_t *)ptr->rx_buf;
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
	  orig=(uint16_t *)buf.rx_buf;
	  copy=(uint16_t *)ptr->rx_buf;
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
_reset( bcu_dev_t *self,
	int        reset_rq )
{
  int rv=0;
  int save_errno=0;
  rv = ioctl( self->fd, reset_rq );
  save_errno = errno;
  bcu_dev_close( self );
  errno = save_errno;
  return rv;
}

static int
_encode_cmd( void            *self,
	     macusb_buffer_t *buf,
	     unsigned int     cmd,
	     uint16_t         attr,
	     uint16_t         data )
{
  return EINVAL;
}

static int
_decode_cmd( void            *self,
	     macusb_buffer_t *buf,
	     unsigned int    *offset,
	     unsigned int     cmd,
	     uint16_t        *state,
	     uint16_t        *attr,
	     uint16_t        *data )
{
  return EINVAL;
}

/*****************************************************************************/
/* Interface */

_bcu_dev_vt_t *
bcu_dev_get_vt( void )
{
  static _bcu_dev_vt_t vt;
  static int is_vt_inited = 0; /* to prevent vt reinitialization during subsequent calls */
  if ( ! is_vt_inited ) {
    vt.encode_cmd = _encode_cmd;
    vt.decode_cmd = _decode_cmd;
    vt.name       = "bcu_dev";
    is_vt_inited = 1;
  }
  return &vt;
}

int
bcu_dev_init( bcu_dev_t *self )
{
  memset( self, 0, sizeof( *self ) );
  self->id = -1;
  self->path = NULL;
  self->fd = -1;
  self->transfer_flags = BCU_DEV_TANSFER_FLAGS;
  self->_vt = *bcu_dev_get_vt();
  return 0;
}

int
bcu_dev_fini( bcu_dev_t *self )
{
  int rv=bcu_dev_close( self );
  self->id = -1;
  return rv;
}

int
bcu_dev_open_id( void         *ptr,
		 unsigned int  dev_id )
{
  int rv=0;
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  int i, idx=-1;
  char pattern[BCU_DEV_PATH_BUFSIZE];
  //char path[BCU_DEV_PATH_BUFSIZE];
  glob_t globbuf;
  struct stat statbuf;
#if defined __USE_MISC || defined __USE_XOPEN2K8
  struct timespec t0, t1;
#else
  struct mytime t0, t1;
#endif
  errno=0;

  memset( &globbuf, 0, sizeof( globbuf ) );
  /* wildcard pattern 'macusb_TYPE_ID-*' matches all DEV devices of TYPE with ID */
  snprintf( pattern, BCU_DEV_PATH_BUFSIZE, BCU_DEV_DEV_DIR"/"MACUSB_NODE_PREFIX"_%s_%u-*", self->_vt.name + strlen( "bcu_" ), dev_id );

  if ( (rv=glob( pattern, GLOB_NOSORT, NULL, &globbuf )) )
    return rv;

  /* macusb_delete() in driver macusb can be run after DEV has been
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
    //snprintf( path, BCU_DEV_PATH_BUFSIZE, "%s", globbuf.gl_pathv[idx] );
    //path[BCU_DEV_PATH_BUFSIZE-1] = 0;
    //strncpy( path, globbuf.gl_pathv[idx], BCU_DEV_PATH_BUFSIZE-1 );
    //if ( !(rv=bcu_dev_open_path( self, path )) )
    if ( !(rv=bcu_dev_open_path( self, globbuf.gl_pathv[idx] )) )
      self->id = dev_id;

  } else {
    rv = ENODEV;
  }
  globfree( &globbuf );
  return rv;
}

int
bcu_dev_open_path( void       *ptr,
		   const char *path )
{
  int rv=0, product_id=0;
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  bcu_dev_close( self );
  if ( !path || !strlen(path) || strlen(path) >= BCU_DEV_PATH_BUFSIZE )
    return 1;
  self->id = -1;
  if ( (self->path=strndup( path, BCU_DEV_PATH_BUFSIZE )) == NULL )
    return ENOMEM;
  if ( (self->fd=open( self->path, BCU_DEV_OPEN_FLAGS )) < 0 )
    return -1;
  if ( (product_id=ioctl( self->fd, MACUSB_IOC_PRODUCT_ID )) == -1 )
    return -1;
  if ( (!strncmp( "bcu_le99s", self->_vt.name, strlen( "bcu_le99s" ) ) &&
	product_id != MACUSB_PRODUCT_ID_LE99S) ||
       (!strncmp( "bcu_le98", self->_vt.name, strlen( "bcu_le98" ) ) &&
	product_id != MACUSB_PRODUCT_ID_LE98) ||
       (!strncmp( "bcu_em11", self->_vt.name, strlen( "bcu_em11" ) ) &&
	product_id != MACUSB_PRODUCT_ID_EM11) )
    return EINVAL;
  return rv;
}

int
bcu_dev_close( void *ptr )
{
  int rv=0;
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  if ( self->fd >= 0 ) {
    rv = close( self->fd );
    self->fd = -1;
  }
  if ( self->path != NULL ) {
    free( self->path );
    self->path = NULL;
  }
  return rv;
}

int
bcu_dev_status( void            *ptr,
		macusb_status_t *st )
{
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  return ioctl( self->fd, MACUSB_IOC_STATUS, st );
}

int
bcu_dev_reset( void *ptr )
{
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  return _reset( self, MACUSB_IOC_RESET_DEVICE );
}

int
bcu_dev_reset_ctl( void *ptr )
{
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  return _reset( self, MACUSB_IOC_RESET_CTL );
}

int
bcu_dev_clear( void *ptr )
{
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  return ioctl( self->fd, MACUSB_IOC_CLEAR );
}

int
bcu_dev_get_sync_cfg( void              *ptr,
		      macusb_sync_cfg_t *cfg )
{
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  return ioctl( self->fd, MACUSB_IOC_GET_SYNC_CFG, cfg );
}

int
bcu_dev_set_sync_cfg( void              *ptr,
		      macusb_sync_cfg_t *cfg )
{
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  return ioctl( self->fd, MACUSB_IOC_SET_SYNC_CFG, cfg );
}

int
bcu_dev_get_async_cfg( void               *ptr,
		       macusb_async_cfg_t *cfg )
{
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  return ioctl( self->fd, MACUSB_IOC_GET_ASYNC_CFG, cfg );
}

int
bcu_dev_set_async_cfg( void               *ptr,
		       macusb_async_cfg_t *cfg )
{
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  return ioctl( self->fd, MACUSB_IOC_SET_ASYNC_CFG, cfg );
}

int
bcu_dev_exec_cmd( void         *ptr,
		  unsigned int  cmd,
		  uint16_t     *state,
		  uint16_t     *attr,
		  uint16_t     *data )
{
  int rv=0;
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  macusb_buffer_t buf;
  unsigned int offset=0;
  memset( &buf, 0, sizeof( buf ) );
  if ( (rv=bcu_dev_encode_cmd( self, &buf, cmd, *attr, *data )) )
    return rv;
  if ( (rv=_exec_buf( self, &buf )) )
    return rv;
  return bcu_dev_decode_cmd( self, &buf, &offset, cmd, state, attr, data );
}

int
bcu_dev_exec_buf( void            *ptr,
		  macusb_buffer_t *buf )
{
  int rv = 0;
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  if ( (rv=_exec_buf( self, buf )) )
    return rv;
  return 0;
}

int
bcu_dev_encode_cmd( void            *ptr,
		    macusb_buffer_t *buf,
		    unsigned int     cmd,
		    uint16_t         attr,
		    uint16_t         data )
{
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  /* virtual funcs commutator */
  return (*self->_vt.encode_cmd)( self, buf, cmd, attr, data );
}

int
bcu_dev_decode_cmd( void            *ptr,
		    macusb_buffer_t *buf,
		    unsigned int    *offset,
		    unsigned int     cmd,
		    uint16_t        *state,
		    uint16_t        *attr,
		    uint16_t        *data )
{
  bcu_dev_t *self = (bcu_dev_t *)ptr;
  /* virtual funcs commutator */
  return (*self->_vt.decode_cmd)( self, buf, offset, cmd, state, attr, data );
}

const char *
bcu_dev_state_str( unsigned int state )
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
