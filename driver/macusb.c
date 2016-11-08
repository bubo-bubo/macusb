/*
 * NOTA BENE:
 *   algo for syscall:
 *        prologue:
 *          1. copy user data if required
 *          2. lock and test if state is valid
 *          3. change state to inform about update operations
 *          4. unlock
 *
 *        
 *          5. make required operations
 *
 *        epilogue:
 *          6. if syscall doesn't require subsequent operations:
 *               lock, restore state, unlock
 */

#include <linux/usb.h>
//#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <asm/uaccess.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/param.h>
#include <linux/sched.h>
#include <linux/jiffies.h>

#include "macusb.h"


MODULE_AUTHOR( "Alexey.Filin@ihep.ru" );
MODULE_DESCRIPTION( "macusb driver" );
MODULE_LICENSE( "GPL" );
MODULE_VERSION( MACUSB_VERSION_STR );

/* table of devices that work with this driver */
static struct usb_device_id macusb_table [] = {
	{ USB_DEVICE( MACUSB_VENDOR_ID, MACUSB_PRODUCT_ID_LE99S ) },
	{ USB_DEVICE( MACUSB_VENDOR_ID, MACUSB_PRODUCT_ID_LE98 ) },
	{ USB_DEVICE( MACUSB_VENDOR_ID, MACUSB_PRODUCT_ID_EM11 ) },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE( usb, macusb_table );


/*****************************************************************************/
#define MACUSB_NAME_MAX_SIZE  256

typedef uint32_t( *mac_func_t )( volatile uint8_t *, uint32_t, uint32_t, uint32_t, macusb_buffer_t * );

struct macusb_bulk_pipe;

/* URB container with attributes
   NB: the container is called URB in the driver */
typedef struct macusb_urb_container {
  struct work_struct       work;  /* container for URB bottom half of
				     completion handler */
  struct list_head         list;
  //volatile uint8_t       *buf;   /* data buffer */
  struct urb              *urb;   /* pointer to URB */
  struct macusb_bulk_pipe *pipe;  /* pointer to URB pipe */
  struct macusb_buffer    *last_rq_buf; /* pointer to last request buffer if
					   not zero, used as flag in RX URB
					   completion handler to change state
					   and wake up sleeping process */
  uint32_t                 size;  /* declared data size */
} macusb_urb_container_t;

/* !!! workqueue( WQ_HIGHPRI ) -- else priority inversion!!! */

struct macusb_dev;

/* Pipe attributes */
typedef struct macusb_bulk_pipe {
  struct list_head             free_urbs;      /* list of free URBs */
  struct list_head             busy_urbs;      /* list of URBs in use */
  struct macusb_dev           *macdev;	       /* pointer to the device */
  struct macusb_pipe_status   *status;         /* pointer to pipe status */
  void (*process_data_locked)(struct macusb_urb_container *); /* data processor in completion handler */
  struct macusb_urb_container *last_uc;        /* last submitted uc or NULL */
  __u8		               bEndpointAddr;  /* endpoint->bEndpointAddress */
  __u16	                       wMaxPacketSize; /* endpoint->wMaxPacketSize */
  unsigned int	               pipe;           /* endpoint "pipe" */
  char                         name[MACUSB_NAME_MAX_SIZE];
} macusb_bulk_pipe_t;

/* Structure to hold all of our device specific stuff */
typedef struct macusb_dev {
  struct macusb_bulk_pipe  bulk_out;  /* bulk-out pipe */
  struct macusb_bulk_pipe  bulk_in;   /* bulk-in pipe */
  struct macusb_sync_cfg   sync_cfg;  /* sync transfer configuration */
  struct macusb_async_cfg  async_cfg; /* async transfer configuration */
  struct macusb_status     status;    /* device status */
  rwlock_t                 rwlock;    /* lock to access the structure */
  struct kref		   kref;
  struct macusb_buffer     last_rq_buf; /* last request buffer to save TX/RX data into */
  struct usb_device       *udev;	 /* the usb device for this device */
  struct usb_interface    *interface; /* the interface for this device */
  atomic_t                 closed;    /* device is single-open */
  char                     name[MACUSB_NAME_MAX_SIZE]; /* devfs node name */
} macusb_dev_t;

/*****************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 11)
static DEFINE_SPINLOCK( macusb_lock );
#else
static spinlock_t macusb_lock = SPIN_LOCK_UNLOCKED;
#endif

static struct usb_driver macusb_driver;

/* overwritten in __init with correct default values */
static macusb_sync_cfg_t macusb_dev_sync_default = {
  .tx_tout = 0,
  .rx_tout = 0,
};

/* overwritten in __init with correct default values */
static macusb_async_cfg_t macusb_dev_async_default = {
  .urb_num  = MACUSB_ASYNC_MAX_URBNUM,
  .mac_tout = MACUSB_ASYNC_MAX_MAC_TOUT,
  .urb_tout = MACUSB_ASYNC_URB_TOUT,
};

/* urb container wait queue */
static DECLARE_WAIT_QUEUE_HEAD(uc_wait_queue);

/* workqueue for bottom half of URB completion handler */
static struct workqueue_struct *bottom_half_workqueue;

/*****************************************************************************/
/* aux funcs */

/* wait conditions */
static int
are_free_urbs( macusb_dev_t       *macdev,
	       macusb_bulk_pipe_t *pipe )
{
  return !list_empty( &macdev->bulk_out.free_urbs ) && !list_empty( &macdev->bulk_in.free_urbs );
}

static int
is_pipe_busy_list_empty( macusb_dev_t       *macdev,
			 macusb_bulk_pipe_t *pipe )
{
  return list_empty( &pipe->busy_urbs );
}

static int
is_rq_completed( macusb_dev_t       *macdev,
		 macusb_bulk_pipe_t *pipe )
{
  return macdev->status.state != MACUSB_STATE_ASYNC_MAC_TOUT;
}

static int
is_rq_stopped( macusb_dev_t       *macdev,
	       macusb_bulk_pipe_t *pipe )
{
  return macdev->status.state != MACUSB_STATE_ASYNC_MAC_TOUT && macdev->status.state != MACUSB_STATE_ASYNC_URB_TOUT;
}

/* interruptible wait for condition with timeout */
static int
wait_for_condition( macusb_dev_t       *macdev,
		    macusb_bulk_pipe_t *pipe,
		    int (*condition)( macusb_dev_t *, macusb_bulk_pipe_t * ) )
{
  int rv=0;
  unsigned long jiffies_tout = jiffies + macdev->async_cfg.urb_tout * HZ / 1000;
  DEFINE_WAIT( last_urbs_wait );
  for (;;) {
    prepare_to_wait( &uc_wait_queue, &last_urbs_wait, TASK_INTERRUPTIBLE );
    if ( (*condition)( macdev, pipe ) )
      break;
    if ( signal_pending( current ) ) {
      rv = -ERESTARTSYS;
      break;
    }
    if ( time_after( jiffies, jiffies_tout ) ) {
      rv = -ETIMEDOUT;
      break;
    }
    write_unlock( &macdev->rwlock );

    schedule_timeout( HZ / 1000 );

    write_lock( &macdev->rwlock );
  }
  finish_wait( &uc_wait_queue, &last_urbs_wait );
  return rv;
}

/* copy data buffer into user space */
static int
copy_data_locked( unsigned long    arg,
		  macusb_buffer_t *buf,
		  macusb_dev_t    *macdev )
{
  int rv=0;
  memcpy( buf, &macdev->last_rq_buf, sizeof( macdev->last_rq_buf ) );
  /* State should be returned to user even with no data in buffer */
  buf->state = macdev->status.state;
  write_unlock( &macdev->rwlock );

  if ( copy_to_user( (void __user *)arg,
		     (const void *)buf,
		     sizeof( *buf ) ) ) {
    rv = -EFAULT;
  }

  write_lock( &macdev->rwlock );
  return rv;
}

/* Abort/cancel transfer requests, clear endpoint halt/stall condition */
static int
pipe_stop_locked( macusb_bulk_pipe_t *pipe )
{
  int rv=0;
  macusb_urb_container_t *pos=NULL;
  /* error while initializing device */
  if ( !pipe->bEndpointAddr )
    return 0;

  /* Abort/cancel transfer requests. Dev lock is hold for asynchronous call
     'usb_unlink_urb' so list 'busy_urbs' is not updated and it is safe to
     traverse the list */
  list_for_each_entry( pos, &pipe->busy_urbs, list ) {
    usb_unlink_urb( pos->urb );
  }

  if ( !(rv=wait_for_condition( pipe->macdev, pipe, is_pipe_busy_list_empty )) ) {
    /* This call is synchronous, and may not be used in an interrupt context. */
    write_unlock( &pipe->macdev->rwlock );

    /* Any URBs queued for such an endpoint should normally be unlinked by the driver before clearing the halt condition, as described in sections 5.7.5 and 5.8.5 of the USB 2.0 spec. */
    usb_clear_halt( pipe->macdev->udev, pipe->pipe );

    write_lock( &pipe->macdev->rwlock );
  }
  return rv;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19)
static void
comp_handler_top_half( struct urb     *urb );
#else
static void
comp_handler_top_half( struct urb     *urb,
		       struct pt_regs *regs );
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
static void
comp_handler_bottom_half( struct work_struct *ptr );
#else
static void
comp_handler_bottom_half( void *ptr );
#endif


static int
pipe_init( macusb_bulk_pipe_t   *pipe,
	   macusb_dev_t         *macdev,
	   macusb_pipe_status_t *status,
	   uint32_t              urb_num,
	   uint32_t              buf_size,
	   uint32_t              is_sndbulkpipe )
{
  int i=0;
  macusb_urb_container_t *uc=NULL;
  void *buf=NULL;
  dma_addr_t dma_addr;
  pipe->macdev = macdev;
  pipe->status = status;
  if ( is_sndbulkpipe )
    pipe->pipe = usb_sndbulkpipe( macdev->udev, pipe->bEndpointAddr );
  else
    pipe->pipe = usb_rcvbulkpipe( macdev->udev, pipe->bEndpointAddr );
  INIT_LIST_HEAD( &pipe->free_urbs );
  INIT_LIST_HEAD( &pipe->busy_urbs );
  for ( i=0; i<urb_num; i++ ) {
    if ( !(uc=kmalloc( sizeof( *uc ), GFP_KERNEL )) ) {
      printk( KERN_ERR "macusb: pipe_init: Could not allocate macusb_urb_container_t" );
      return -ENOMEM;
    }
    memset( uc, 0, sizeof( *uc ) );
    uc->pipe = pipe;
    if ( ! (uc->urb=usb_alloc_urb( 0, GFP_KERNEL )) ){
      kfree( uc );
      printk( KERN_ERR "macusb: pipe_init: Could not allocate 'struct urb'" );
      return -ENOMEM;
    }
    uc->urb->context = uc;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
    buf = (uint8_t *)usb_alloc_coherent( macdev->udev,
					 buf_size,
					 GFP_KERNEL,
					 &dma_addr );
#else
    buf = (uint8_t *)usb_buffer_alloc( macdev->udev,
				       buf_size,
				       GFP_KERNEL,
				       &dma_addr );
#endif
    if ( !buf ){
      kfree( uc );
      printk( KERN_ERR "macusb: pipe_init: Could not allocate urb transfer buffer" );
      return -ENOMEM;
    }
    usb_fill_bulk_urb( uc->urb,
		       macdev->udev,
		       pipe->pipe,
		       buf,
		       buf_size,
		       comp_handler_top_half,
		       uc );

    uc->urb->transfer_dma = dma_addr;
    uc->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP; /* coherent DMA */

    INIT_LIST_HEAD( &uc->list );
    list_add_tail( &uc->list, &pipe->free_urbs );
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
    INIT_WORK( &uc->work, comp_handler_bottom_half );
#else
    INIT_WORK( &uc->work, comp_handler_bottom_half, NULL );
#endif
  }
  return 0;
}

static void
pipe_fini_locked( macusb_bulk_pipe_t *pipe,
		  uint32_t            size )
{
  macusb_urb_container_t *pos=NULL, *next=NULL;
  /* error while initializing device */
  if ( !pipe->bEndpointAddr )
    return;

  /* all URBs finished so can be deleted from free_urbs list */
  list_for_each_entry_safe( pos, next, &pipe->free_urbs, list ) {
    list_del( &pos->list );
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
    usb_free_coherent( pipe->macdev->udev,
		       size,
		       pos->urb->transfer_buffer,
		       pos->urb->transfer_dma );
#else
    usb_buffer_free( pipe->macdev->udev,
		     size,
		     pos->urb->transfer_buffer,
		     pos->urb->transfer_dma );
#endif
    usb_free_urb( pos->urb );
    kfree( pos );
  }
}

/*****************************************************************************/
/* driver commands */

static int
device_id( macusb_dev_t *macdev )
{
  int rv=0;
  struct usb_device_descriptor dd;

  memset( &dd, 0, sizeof( dd ) );
  if ( (rv=usb_get_descriptor( macdev->udev,
			       USB_DT_DEVICE,
			       0,
			       &dd,
			       sizeof( dd ) )) > 0 ) {
    union {
      char           c[2];
      unsigned short v;
    } bcd;
    bcd.v = dd.bcdDevice; /* little-endian */
    //rv = (int)( (bcd.c[0] & 0xf) +
    //		((bcd.c[0] & 0xf0) >> 4) * 10 +
    //		(bcd.c[1] & 0xf)  * 100 +
    //		((bcd.c[1] & 0xf0) >> 4) * 1000 ); /* convert to decimal */
    rv = dd.bcdDevice;

  } else if ( rv == 0 ) {
    rv = -ENODEV;
  }
  return rv;
}

static int
device_product_id( macusb_dev_t *macdev )
{
  return macdev->udev->descriptor.idProduct;
}

static int
device_reset( macusb_dev_t *macdev )
{
  int rv=0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
  usb_queue_reset_device( macdev->interface );
#else
  /* usb_reset_device: For calls that might not occur during probe, drivers should lock the device using usb_lock_device_for_reset. */
  if ( (rv=usb_lock_device_for_reset( macdev->udev, macdev->interface )) == 0 ) {
    rv = usb_reset_device( macdev->udev );
    usb_unlock_device( macdev->udev );
  }
#endif
  return rv;
}

static int
device_reset_ctl( macusb_dev_t *macdev )
{
  return usb_control_msg( macdev->udev,
			  usb_sndctrlpipe( macdev->udev, 0 ),
			  USB_REQ_CLEAR_FEATURE,
			  USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT,
			  0, 0,
			  NULL, 0, HZ );
}

/* clear states, counters, buffers */
static int
device_clear( macusb_dev_t *macdev )
{
  int rv=0;

  write_lock( &macdev->rwlock );
  if ( macdev->status.state != MACUSB_STATE_FREE &&
       macdev->status.state != MACUSB_STATE_ERR ) {
    rv = -EINPROGRESS;

  } else {
    memset( &macdev->status, 0, sizeof( macdev->status ) );
    memset( &macdev->last_rq_buf, 0, sizeof( macdev->last_rq_buf ) );
    macdev->status.state = MACUSB_STATE_FREE;
  }
  write_unlock( &macdev->rwlock );
  return rv;
}

static int
device_get_sync_cfg( macusb_dev_t  *macdev,
		     unsigned long  arg )
{
  macusb_sync_cfg_t cfg;

  read_lock( &macdev->rwlock );
  cfg = macdev->sync_cfg;
  read_unlock( &macdev->rwlock );

  if ( copy_to_user( (void __user *)arg,
		     (const void *)&cfg,
		     sizeof( cfg ) ) )
    return -EFAULT;

  return 0;
}

static int
set_sync_cfg( macusb_dev_t      *macdev,
	      macusb_sync_cfg_t *cfg )
{
  int rv=0;

  macdev->sync_cfg.tx_tout = cfg->tx_tout;
  macdev->sync_cfg.rx_tout = cfg->rx_tout;

  return rv;
}

static int
device_set_sync_cfg( macusb_dev_t  *macdev,
		     unsigned long  arg )
{
  int rv=0;
  macusb_sync_cfg_t cfg;

  if ( copy_from_user( (void *)&cfg,
		       (const void __user *)arg,
		       sizeof( cfg ) ) ) {
    rv = -EFAULT;

  } else {
    write_lock( &macdev->rwlock );
    if ( macdev->status.state != MACUSB_STATE_FREE ) {
      rv = -EINPROGRESS;

    } else {
      rv = set_sync_cfg( macdev, &cfg );
    }
    write_unlock( &macdev->rwlock );
  }
  return rv;
}

static int
device_get_async_cfg( macusb_dev_t  *macdev,
		      unsigned long  arg )
{
  macusb_async_cfg_t cfg;

  read_lock( &macdev->rwlock );
  cfg = macdev->async_cfg;
  read_unlock( &macdev->rwlock );

  if ( copy_to_user( (void __user *)arg,
		     (const void *)&cfg,
		     sizeof( cfg ) ) )
    return -EFAULT;

  return 0;
}

static int
set_async_cfg( macusb_dev_t       *macdev,
	       macusb_async_cfg_t *cfg )
{
  int rv=0;

  /* DON'T TOUCH!!! */
  macdev->async_cfg = *cfg;

  if ( (rv=pipe_init( &macdev->bulk_out, macdev, &macdev->status.bulk_out, macdev->async_cfg.urb_num, MACUSB_TXBUFSIZE, 1 )) )
    return rv;
  if ( (rv=pipe_init( &macdev->bulk_in, macdev, &macdev->status.bulk_in, macdev->async_cfg.urb_num, MACUSB_RXBUFSIZE, 0 )) )
    return rv;

  return rv;
}

static int
device_set_async_cfg( macusb_dev_t  *macdev,
		      unsigned long  arg )
{
  int rv=0;
  macusb_async_cfg_t cfg;

  if ( copy_from_user( (void *)&cfg,
		       (const void __user *)arg,
		       sizeof( cfg ) ) ) {
    rv = -EFAULT;

  } else {
    write_lock( &macdev->rwlock );
    if ( macdev->status.state != MACUSB_STATE_FREE ) {
      rv = -EINPROGRESS;

    } else {
      if ( !cfg.urb_num ||
	   cfg.urb_num > MACUSB_ASYNC_MAX_URBNUM ||
	   cfg.mac_tout > MACUSB_ASYNC_MAX_MAC_TOUT ) {
	rv = -EINVAL;

      } else {
	macdev->status.state = MACUSB_STATE_CONF;
	pipe_fini_locked( &macdev->bulk_out, MACUSB_TXBUFSIZE );
	pipe_fini_locked( &macdev->bulk_in, MACUSB_RXBUFSIZE );
	rv = set_async_cfg( macdev, &cfg );
	macdev->status.state = MACUSB_STATE_FREE;
      }
    }
    write_unlock( &macdev->rwlock );
  }
  return rv;
}

static int
device_status( macusb_dev_t  *macdev,
	       unsigned long  arg )
{
  macusb_status_t st;

  memset( &st, 0, sizeof( st ) );

  read_lock( &macdev->rwlock );
  st = macdev->status;
  read_unlock( &macdev->rwlock );

  if ( copy_to_user( (void __user *)arg,
		     (const void *)&st,
		     sizeof( st ) ) )
    return -EFAULT;

  return 0;
}

static int
_device_rx( macusb_dev_t  *macdev,
	    unsigned long  arg,
	    mac_func_t     rx_mac )
{
  int rv=0;
  macusb_buffer_t buf;
  /* copy iface structure from user space */
  if ( copy_from_user( (void *)&buf,
		       (const void __user *)arg,
		       sizeof( buf ) ) ) {
    rv = -EFAULT;

    /* test TX/RX buffer size */
  } else if ( buf.tx_bytes > MACUSB_TXBUFSIZE ||
	      buf.rx_eot_offset + sizeof( MACUSB_EOT_DATATYPE ) > MACUSB_RXBUFSIZE ) { /* controller returns <=508 bytes so RQ_ECHO should work correctly */
    rv = -EINVAL;

  } else {
    write_lock( &macdev->rwlock );
    if ( macdev->status.state != MACUSB_STATE_FREE ) {
      /* request in progress, failed or device disconnected */
      rv = -EPERM;

    } else {
      /* device node is ready to process request */
      macdev->status.state = MACUSB_STATE_ASYNC_BUSY;
      macdev->status.err = 0;
      if ( !(*are_free_urbs)( macdev, NULL ) &&
	   (rv=wait_for_condition( macdev, NULL, are_free_urbs )) ) {
	if ( macdev->status.state != MACUSB_STATE_ASYNC_BUSY ) {
	  /* stopped, disconnected */
	  rv = -ECANCELED;

	} else {
	  /* timed out, interrupted */
	  macdev->status.state = MACUSB_STATE_FREE;
	}
      }
      if ( !rv ) {
	struct list_head *next=NULL;
	macusb_urb_container_t *uc=NULL;

	memcpy( &macdev->last_rq_buf, &buf, sizeof( buf ) );
	macdev->last_rq_buf.rx_bytes = 0;
	macdev->last_rq_buf.rx_urbstatus = 0;

	next = macdev->bulk_out.free_urbs.next;
	uc = container_of( next, macusb_urb_container_t, list );
	//memcpy( (void *)uc->buf, macdev->last_rq_buf.tx_buf, macdev->last_rq_buf.tx_bytes );
	memcpy( (void *)uc->urb->transfer_buffer, macdev->last_rq_buf.tx_buf, macdev->last_rq_buf.tx_bytes );
	uc->size = uc->urb->transfer_buffer_length = buf.tx_bytes;

	if ( !(rv=usb_submit_urb( uc->urb, GFP_ATOMIC )) ) {
	  unsigned int bin_idx = 0;
	  list_move_tail( next, &macdev->bulk_out.busy_urbs );
	  macdev->bulk_out.last_uc = uc;
	  macdev->bulk_out.status->busy_urbs++;
	  bin_idx = MACUSB_URB_OCCUPANCY_HISTO_SIZE * macdev->bulk_out.status->busy_urbs / macdev->async_cfg.urb_num;
	  bin_idx = (bin_idx < MACUSB_URB_OCCUPANCY_HISTO_SIZE) ? bin_idx : MACUSB_URB_OCCUPANCY_HISTO_SIZE - 1;
	  macdev->bulk_out.status->urb_occupancy[bin_idx]++;

	  /* request URB has been submitted. Submit answer URB */
	  next = macdev->bulk_in.free_urbs.next;
	  uc = container_of( next, macusb_urb_container_t, list );
	  uc->size = 0;
	  uc->last_rq_buf = NULL;
	  //memset( (void *)uc->buf, 0, MACUSB_RXBUFSIZE ); /* NB: Clear RX buffer for MAC! */
	  memset( (void *)uc->urb->transfer_buffer, 0, MACUSB_RXBUFSIZE ); /* NB: Clear RX buffer for MAC! */
	  if ( !(rv=usb_submit_urb( uc->urb, GFP_ATOMIC )) ) {
	    unsigned long rx_bytes=0;
	    list_move_tail( next, &macdev->bulk_in.busy_urbs );
	    macdev->bulk_in.last_uc = uc;
	    macdev->bulk_in.status->busy_urbs++;
	    bin_idx = MACUSB_URB_OCCUPANCY_HISTO_SIZE * macdev->bulk_in.status->busy_urbs / macdev->async_cfg.urb_num;
	    bin_idx = (bin_idx < MACUSB_URB_OCCUPANCY_HISTO_SIZE) ? bin_idx : MACUSB_URB_OCCUPANCY_HISTO_SIZE - 1;
	    macdev->bulk_in.status->urb_occupancy[bin_idx]++;
	    /* run MAC */
	    //if ( (rx_bytes=(*rx_mac)( (uint8_t *)uc->buf, MACUSB_RXBUFSIZE, macdev->async_cfg.mac_tout, &macdev->last_rq_buf )) ) {
	    if ( (rx_bytes=(*rx_mac)( (uint8_t *)uc->urb->transfer_buffer,
				      MACUSB_RXBUFSIZE,
				      macdev->last_rq_buf.rx_eot_offset,
				      macdev->async_cfg.mac_tout,
				      &macdev->last_rq_buf )) ) {

	      /* MAC finished request successfully */
	      //memcpy( macdev->last_rq_buf.rx_buf, (void *)uc->buf, rx_bytes );
	      memcpy( macdev->last_rq_buf.rx_buf, (void *)uc->urb->transfer_buffer, rx_bytes );
	      macdev->last_rq_buf.rx_bytes = rx_bytes;
	      uc->size = rx_bytes;
	      macdev->status.state = MACUSB_STATE_FREE;

	    } else {
	      /* MAC timeout or broken answer. Wait for RX completion handler */
	      uc->last_rq_buf = &macdev->last_rq_buf;
	      macdev->bulk_in.status->mac_tout_urbs++;
	      macdev->status.state = MACUSB_STATE_ASYNC_MAC_TOUT;
	      if ( (rv=wait_for_condition( macdev, NULL, is_rq_completed )) ) {
		if ( rv == -ETIMEDOUT )
		  /* URB timeout */
		  macdev->status.state = MACUSB_STATE_ASYNC_URB_TOUT;
		else
		  macdev->status.state = MACUSB_STATE_ERR;
		if ( rv == -ERESTARTSYS )
		  rv = -EINTR;

	      } else if ( macdev->status.state != MACUSB_STATE_FREE ) {
		if ( macdev->status.err ) {
		  /* if URB fails completion handler sets error */
		  rv = macdev->status.err;
		} else {
		  rv = -EIO;
		}
	      }
	    }
	    /* copy RX data into user buffer */
	    if ( !rv )
	      rv = copy_data_locked( arg, &buf, macdev );

	  } else {
	    /* It is impossible to conclude device caused the problem or
	       host did.

	       NB: State is changed forcibly so user should at least clear
	       device to erase possibly suspended bulk-out URB.
	       Clearing is safe for all finished requests because
	       completion handler doesn't change RX data of MAC'ed
	       RX URBs. It is up to user to reset device or not
	       somehow. */
	    macdev->status.state = MACUSB_STATE_ERR;
	  }
	} else {
	  /* It is impossible to conclude device caused the problem or
	     host did.

	     NB: It is up to user to reset device or not */
	}
      }
    }
    macdev->status.err = rv;
    write_unlock( &macdev->rwlock );
  }
  return rv;
}

/* MAC for request with pointed length answer */
static uint32_t
_rx_mac( volatile uint8_t *rx_ptr,
	 uint32_t          rx_size,
	 uint32_t          rx_eot_offset,
	 uint32_t          usec,
	 macusb_buffer_t  *last_rq_buf )
{
  volatile MACUSB_EOT_DATATYPE *eot=(MACUSB_EOT_DATATYPE *)(rx_ptr + rx_eot_offset);
  uint32_t counter=0;
  while ( !*eot ) {
    if ( counter++ > usec )
      break;
    udelay( 1 ); /* Hard-coded, counter should be measured in usec.
		    Is sub-usec granularity required? */
  }
  /* verify end-of-transfer mark */
  if ( *eot == MACUSB_EOT )
    return rx_eot_offset + sizeof( MACUSB_EOT_DATATYPE );
  /* either timeout or incorrect packet */
  return 0;
}

/* MAC for request with variable length answer: N 16-bit words */
static uint32_t
_n16_rx_mac( volatile uint8_t *rx_ptr,
	     uint32_t          rx_size,
	     uint32_t          rx_eot_offset,
	     uint32_t          usec,
	     macusb_buffer_t  *last_rq_buf )
{
  volatile MACUSB_EOT_DATATYPE *eot=&((macusb_rq_rx_prepacket_t *)rx_ptr)->eot;
  uint32_t counter=0;
  uint32_t data_len=0;
  unsigned int i=0;
  union {
    macusb_rq_rx_prepacket_t bits;
    uint16_t                 buf[sizeof(macusb_rq_rx_prepacket_t)/2];
  } prepacket;

  while ( !*eot ) {
    if ( counter++ > usec )
      break;
    udelay( 1 ); /* Hard-coded, counter should be measured in usec.
		    Is sub-usec granularity required? */
  }
  /* either timeout or incorrect packet */
  if ( *eot != MACUSB_EOT )
    return 0;
  /* data length in 16-bit words should be converted to host byte order
     NB: bus controller is little-endian device, change conversion if not */
  //data_len = ((macusb_rq_rx_prepacket_t *)rx_ptr)->len;
  for ( i=0; i<sizeof(macusb_rq_rx_prepacket_t)/2; i++ )
    prepacket.buf[i] = le16_to_cpu( *((uint16_t *)(rx_ptr + i * 2)) );
  data_len = prepacket.bits.len;
  /* too many data or incorrect packet */
  if ( sizeof( macusb_rq_rx_prepacket_t ) + data_len * sizeof(uint16_t) + sizeof( MACUSB_EOT_DATATYPE ) > rx_size )
    return 0;
  rx_ptr += sizeof( macusb_rq_rx_prepacket_t );
  eot = (uint32_t *)(rx_ptr + sizeof( macusb_rq_rx_prepacket_t ) + data_len * sizeof(uint16_t));
  counter = 0;
  while ( !*eot ) {
    if ( counter++ > usec )
      break;
    udelay( 1 ); /* Hard-coded, counter should be measured in usec.
		    Is sub-usec granularity required? */
  }
  /* either timeout or incorrect packet */
  if ( *eot != MACUSB_EOT )
    return 0;
  /* sizeof first packet + data length + sizeof end-of-transfer mark */
  return sizeof( macusb_rq_rx_prepacket_t ) + data_len * sizeof( uint16_t ) + sizeof( MACUSB_EOT_DATATYPE );
}

/* MAC for request with variable length answer: N 32-bit words */
static uint32_t
_n32_rx_mac( volatile uint8_t *rx_ptr,
	     uint32_t          rx_size,
	     uint32_t          rx_eot_offset,
	     uint32_t          usec,
	     macusb_buffer_t  *last_rq_buf )
{
  volatile MACUSB_EOT_DATATYPE *eot=&((macusb_rq_rx_prepacket_t *)rx_ptr)->eot;
  uint32_t counter=0;
  uint32_t data_len=0;
  unsigned int i=0;
  union {
    macusb_rq_rx_prepacket_t bits;
    uint16_t                 buf[sizeof(macusb_rq_rx_prepacket_t)/2];
  } prepacket;

  while ( !*eot ) {
    if ( counter++ > usec )
      break;
    udelay( 1 ); /* Hard-coded, counter should be measured in usec.
		    Is sub-usec granularity required? */
  }
  /* either timeout or incorrect packet */
  if ( *eot != MACUSB_EOT )
    return 0;
  /* data length in 32-bit words should be converted to host byte order
     NB: bus controller is little-endian device, change conversion if not */
  //data_len = ((macusb_rq_rx_prepacket_t *)rx_ptr)->len;
  for ( i=0; i<sizeof(macusb_rq_rx_prepacket_t)/2; i++ )
    prepacket.buf[i] = le16_to_cpu( *((uint16_t *)(rx_ptr + i * 2)) );
  data_len = prepacket.bits.len;
  /* too many data or incorrect packet */
  if ( sizeof( macusb_rq_rx_prepacket_t ) + data_len * sizeof(uint32_t) + sizeof( MACUSB_EOT_DATATYPE ) > rx_size )
    return 0;
  rx_ptr += sizeof( macusb_rq_rx_prepacket_t );
  eot = (uint32_t *)(rx_ptr + sizeof( macusb_rq_rx_prepacket_t ) + data_len * sizeof(uint32_t));
  counter = 0;
  while ( !*eot ) {
    if ( counter++ > usec )
      break;
    udelay( 1 ); /* Hard-coded, counter should be measured in usec.
		    Is sub-usec granularity required? */
  }
  /* either timeout or incorrect packet */
  if ( *eot != MACUSB_EOT )
    return 0;
  /* sizeof first packet + data length + sizeof end-of-transfer mark */
  return sizeof( macusb_rq_rx_prepacket_t ) + data_len * sizeof( uint32_t ) + sizeof( MACUSB_EOT_DATATYPE );
}

/* MAC for request with echo answer */
static uint32_t
_echo_rx_mac( volatile uint8_t *rx_ptr,
	      uint32_t          rx_size,
	      uint32_t          rx_eot_offset,
	      uint32_t          usec,
	      macusb_buffer_t  *last_rq_buf )
{
  volatile uint8_t *eot=rx_ptr + last_rq_buf->tx_bytes - 1;
  uint32_t counter=0;
  uint8_t last_byte = *(last_rq_buf->tx_buf + last_rq_buf->tx_bytes - 1);
  while ( *eot != last_byte ) {
    if ( counter++ > usec )
      break;
    udelay( 1 ); /* Hard-coded, counter should be measured in usec.
		    Is sub-usec granularity required? */
  }
  /* verify last byte */
  if ( *eot == last_byte )
    return last_rq_buf->tx_bytes;
  /* either timeout or incorrect packet */
  return 0;
}

static int
device_rq_eot( macusb_dev_t  *macdev,
	       unsigned long  arg )
{
  return _device_rx( macdev, arg, _rx_mac );
}

static int
device_rq_n16( macusb_dev_t  *macdev,
	       unsigned long  arg )
{
  return _device_rx( macdev, arg, _n16_rx_mac );
}

static int
device_rq_n32( macusb_dev_t  *macdev,
	       unsigned long  arg )
{
  return _device_rx( macdev, arg, _n32_rx_mac );
}

static int
device_rq_echo( macusb_dev_t  *macdev,
		unsigned long  arg )
{
  return _device_rx( macdev, arg, _echo_rx_mac );
}

static int
device_stop_rq( macusb_dev_t  *macdev,
		unsigned long  arg )
{
  int rv=0;
  macusb_status_t st;

  write_lock( &macdev->rwlock );
  if ( macdev->status.state == MACUSB_STATE_ASYNC_MAC_TOUT ||
       macdev->status.state == MACUSB_STATE_ASYNC_URB_TOUT ) {
    if ( macdev->bulk_out.last_uc )
      usb_unlink_urb( macdev->bulk_out.last_uc->urb );
    if ( macdev->bulk_in.last_uc )
      usb_unlink_urb( macdev->bulk_in.last_uc->urb );
    rv = wait_for_condition( macdev, NULL, is_rq_stopped );
    macdev->status.state = MACUSB_STATE_ERR;
    macdev->status.err = -EIO;

  } else if ( macdev->status.state == MACUSB_STATE_SYNC_BUSY ) {
    macdev->status.state = MACUSB_STATE_ERR;
    macdev->status.err = -EIO;
  }
  st = macdev->status;
  write_unlock( &macdev->rwlock );

  if ( !rv && copy_to_user( (void __user *)arg,
			    (const void *)&st,
			    sizeof( st ) ) )
    return -EFAULT;
  return rv;
}

static int
device_copy_rq( macusb_dev_t  *macdev,
		unsigned long  arg )
{
  int rv=0;
  macusb_buffer_t buf;

  write_lock( &macdev->rwlock );
  if ( macdev->status.state == MACUSB_STATE_SYNC_BUSY ||
       macdev->status.state == MACUSB_STATE_SYNC_READ ||
       macdev->status.state == MACUSB_STATE_ASYNC_MAC_TOUT ||
       macdev->status.state == MACUSB_STATE_ASYNC_URB_TOUT ) {
    /* RX in progress */
    rv = -EPERM;

  } else {
    rv = copy_data_locked( arg, &buf, macdev );
  }
  write_unlock( &macdev->rwlock );
 
  return rv;
}


/*****************************************************************************/
/* driver interface */

static void
macusb_delete( struct kref *kref )
{	
  macusb_dev_t *macdev = container_of( kref, macusb_dev_t, kref );
  write_lock( &macdev->rwlock );
  macdev->status.state = MACUSB_STATE_END; /* notify completion handlers */
  pipe_stop_locked( &macdev->bulk_out );
  pipe_fini_locked( &macdev->bulk_out, MACUSB_TXBUFSIZE );
  pipe_stop_locked( &macdev->bulk_in );
  pipe_fini_locked( &macdev->bulk_in, MACUSB_RXBUFSIZE );
  write_unlock( &macdev->rwlock );

  usb_put_dev( macdev->udev );

  kfree( macdev );
}

static int
macusb_open( struct inode *inode,
	     struct file  *file )
{
  macusb_dev_t *macdev = NULL;
  struct usb_interface *interface = NULL;
  int subminor;
  int retval = 0;

  subminor = iminor( inode );

  interface = usb_find_interface( &macusb_driver, subminor );
  if ( !interface ) {
    printk( KERN_ERR "macusb: %s - error, can't find device for minor %d",
	 __FUNCTION__, subminor );
    retval = -ENODEV;

  } else {
    /* prevent macusb_open() from racing macusb_disconnect() */
    spin_lock( &macusb_lock );
    macdev = usb_get_intfdata( interface );

    if ( !macdev ) {
      retval = -ENODEV;

    } else {
      if ( ! atomic_dec_and_test( &macdev->closed ) ) {
	atomic_inc( &macdev->closed );
	retval = -EBUSY; /* already open */

      } else {
	/* increment our usage count for the device */
	kref_get( &macdev->kref );

	/* save our object in the file's private structure */
	file->private_data = macdev;
      }
    }
    spin_unlock( &macusb_lock );
  }
#ifdef MACUSB_DBG
printk( KERN_INFO "macusb_open, name='%s'\n", macdev->name );
#endif
  return retval;
}

static int
macusb_release( struct inode *inode,
		struct file  *file )
{
  macusb_dev_t *macdev;
  int subminor;

  subminor = iminor( inode );

  spin_lock( &macusb_lock );
  macdev = (macusb_dev_t *)file->private_data;
  if ( atomic_read( &macdev->closed ) ) {
    spin_unlock( &macusb_lock );
    /* not open */
    return -ENODEV;
  }
#ifdef MACUSB_DBG
printk( KERN_INFO "macusb_release, name='%s'\n", macdev->name );
#endif
  atomic_inc( &macdev->closed ); /* release the device */
  spin_unlock( &macusb_lock );

  kref_put( &macdev->kref, macusb_delete );
  return 0;
}

static ssize_t
macusb_write( struct file *file,
	      const char  *buffer,
	      size_t       count,
	      loff_t      *ppos )
{
  macusb_dev_t *macdev;
  int rv=0;
  int cnt=0;
  macusb_buffer_t buf;

  macdev = (macusb_dev_t *)file->private_data;

  if ( count > MACUSB_TXBUFSIZE )
    return -EINVAL;

  memset( &buf, 0, sizeof( buf ) );
  if ( copy_from_user( (void *)buf.tx_buf,
		       (const void __user *)buffer,
		       count ) ) {
    return -EFAULT;
  }
  buf.tx_bytes = count;

  write_lock( &macdev->rwlock );
  if ( macdev->status.state != MACUSB_STATE_FREE ) {
    write_unlock( &macdev->rwlock );
    return -EPERM;
  }
  memcpy( macdev->last_rq_buf.tx_buf, buf.tx_buf, count );
  macdev->last_rq_buf.tx_bytes = count;
  macdev->status.state = MACUSB_STATE_SYNC_WRITE;
  write_unlock( &macdev->rwlock );

  if ( ! (rv=usb_bulk_msg( macdev->udev,
			   macdev->bulk_out.pipe,
			   buf.tx_buf,
			   count,
			   &cnt,
			   macdev->sync_cfg.tx_tout * HZ / 1000 )) ) {
    macdev->status.bulk_out.bytes += cnt;
  }

  write_lock( &macdev->rwlock );
  if ( rv == 0 ) {
    macdev->status.state = MACUSB_STATE_SYNC_BUSY;
    rv = cnt;

  } else {
    macdev->status.state = MACUSB_STATE_ERR;
  }
  write_unlock( &macdev->rwlock );

  return rv;
}

static ssize_t
macusb_read( struct file *file,
	     char        *buffer,
	     size_t       count,
	     loff_t      *ppos )
{
  macusb_dev_t *macdev;
  int rv=0;
  macusb_buffer_t buf;

  macdev = (macusb_dev_t *)file->private_data;

  if ( count < MACUSB_RXBUFSIZE )
    return -EINVAL;

  write_lock( &macdev->rwlock );
  if ( macdev->status.state != MACUSB_STATE_SYNC_BUSY ) {
    write_unlock( &macdev->rwlock );
    return -EPERM;
  }
  macdev->status.state = MACUSB_STATE_SYNC_READ;
  write_unlock( &macdev->rwlock );

  memset( &buf, 0, sizeof( buf ) );
  rv = usb_bulk_msg( macdev->udev,
		     macdev->bulk_in.pipe,
		     buf.rx_buf,
		     MACUSB_RXBUFSIZE,
		     &buf.rx_bytes,
		     macdev->sync_cfg.rx_tout * HZ / 1000 );

  write_lock( &macdev->rwlock );
  if ( rv == 0 ) {
    if ( buf.rx_bytes ) {
      memcpy( macdev->last_rq_buf.rx_buf, buf.rx_buf, buf.rx_bytes );
      macdev->last_rq_buf.rx_bytes = buf.rx_bytes;
      macdev->status.bulk_in.bytes += buf.rx_bytes;
    }
    macdev->status.state = MACUSB_STATE_FREE;

  } else {
    macdev->status.err = rv;
    macdev->last_rq_buf.rx_urbstatus = rv;
    macdev->status.state = MACUSB_STATE_ERR;
  }
  write_unlock( &macdev->rwlock );

  if ( !rv ) {
    rv = buf.rx_bytes;
    if ( copy_to_user( (void __user *)buffer,
		       (const void *)buf.rx_buf,
		       buf.rx_bytes ) ) {
      rv = -EFAULT;
    }
  }
  return rv;
}

/* data processor in RX completion handler */
static void
process_data_locked( macusb_urb_container_t *uc )
{
  if ( uc->last_rq_buf ) {
    /* after MAC timeout */
    macusb_dev_t *macdev = uc->pipe->macdev;
    if ( macdev->status.state != MACUSB_STATE_ASYNC_MAC_TOUT &&
	 macdev->status.state != MACUSB_STATE_ASYNC_URB_TOUT )
      /* MAC succeeded, request interrupted, TX failed, no processing is required */
      return;
    //memcpy( dev->last_rq_buf.rx_buf, (void *)uc->buf, uc->urb->actual_length );
    if ( uc->urb->actual_length ) {
      memcpy( macdev->last_rq_buf.rx_buf, (void *)uc->urb->transfer_buffer, uc->urb->actual_length );
    }
    macdev->last_rq_buf.rx_bytes = uc->urb->actual_length;
    if ( !uc->urb->status ) {
      macdev->status.state = MACUSB_STATE_FREE;
    } else {
      macdev->last_rq_buf.rx_urbstatus = uc->urb->status;
      macdev->status.state = MACUSB_STATE_ERR;
      macdev->status.err = uc->urb->status;
    }
  }
}

/* completion handler bottom half */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
static void
comp_handler_bottom_half( struct work_struct *ptr )
#else
static void
comp_handler_bottom_half( void *ptr )
#endif
{
  macusb_bulk_pipe_t *pipe=NULL;
  macusb_dev_t *macdev=NULL;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
  macusb_urb_container_t *uc=container_of( ptr, macusb_urb_container_t, work );
#else
  macusb_urb_container_t *uc=(macusb_urb_container_t *)ptr;
#endif
  if ( uc == NULL )
    return;
  pipe = uc->pipe;
  macdev = pipe->macdev;
//printk( KERN_INFO "comp_handler_bottom_half, urb->status=%d, urb->actual_length=%d\n", urb->status, urb->actual_length );
  /* experimental fact: write_lock can lock cpu on overloaded smp system
     indefinitely long if RQ syscalls are called by a user process
     without delays. A lock in workqueue is traced and reported by kernel. */
  //write_lock( &dev->rwlock );
  if ( !write_trylock( &macdev->rwlock ) ) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
    PREPARE_WORK( &uc->work, comp_handler_bottom_half );
#else
    PREPARE_WORK( &uc->work, comp_handler_bottom_half, uc );
#endif
    queue_work( bottom_half_workqueue, &uc->work );
    return;
  }
  /* RX: copy data, change state if required */
  if ( uc->pipe->process_data_locked )
    (*uc->pipe->process_data_locked)( uc );
  /* update stats */
  pipe->status->bytes += uc->urb->actual_length;
  pipe->status->busy_urbs--;
  pipe->status->completed_urbs++;
  /* TX */
  if ( uc->size && uc->urb->actual_length != uc->size ) {
    macdev->status.state = MACUSB_STATE_ERR;
    macdev->status.err = -EIO;
    pipe->status->size_mismatch_urbs++;
  }
  if ( uc->urb->status ) {
    pipe->status->bad_status_urbs++;
  }
  uc->last_rq_buf = NULL;
  list_move_tail( &uc->list, &pipe->free_urbs );
  if ( pipe->last_uc == uc )
    pipe->last_uc = NULL;
  write_unlock( &macdev->rwlock );
  /* wake up threads sleeping in syscalls, concurrent control syscalls e.g.
     device_clear are to be woken up always */
  wake_up_interruptible_all( &uc_wait_queue );
}

/* completion handler top half */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19)
static void
comp_handler_top_half( struct urb     *urb )
#else
static void
comp_handler_top_half( struct urb     *urb,
		       struct pt_regs *regs )
#endif
{
  macusb_urb_container_t *uc=NULL;

  if ( urb == NULL )
    return;
  uc = (macusb_urb_container_t *)urb->context;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
  PREPARE_WORK( &uc->work, comp_handler_bottom_half );
#else
  PREPARE_WORK( &uc->work, comp_handler_bottom_half, uc );
#endif
  queue_work( bottom_half_workqueue, &uc->work );
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 11)
static long
macusb_ioctl( struct file   *file,
	      unsigned int   cmd,
	      unsigned long  arg )
#else
static int
macusb_ioctl( struct inode  *inode,
	      struct file   *file,
	      unsigned int   cmd,
	      unsigned long  arg )
#endif
{
  macusb_dev_t *macdev;
  int retval=0;

  if ( _IOC_TYPE( cmd ) != MACUSB_IOC_MAGIC ) return -ENOTTY;
  if ( _IOC_NR( cmd ) > MACUSB_IOC_MAXNR ) return -ENOTTY;

  macdev = (macusb_dev_t *)file->private_data;
  if ( macdev == NULL )
    return -ENODEV;

  switch ( cmd ) {
/* control */
  case MACUSB_IOC_DEVICE_ID:
    retval = device_id( macdev );
    break;
  case MACUSB_IOC_PRODUCT_ID:
    retval = device_product_id( macdev );
    break;
  case MACUSB_IOC_RESET_DEVICE:
    retval = device_reset( macdev );
    break;
  case MACUSB_IOC_RESET_CTL:
    retval = device_reset_ctl( macdev );
    break;
  case MACUSB_IOC_CLEAR:
#ifdef MACUSB_DBG
printk( KERN_INFO "CLEAR, name='%s'\n", macdev->name );
#endif
    retval = device_clear( macdev );
    break;
  case MACUSB_IOC_GET_SYNC_CFG:
    retval = device_get_sync_cfg( macdev, arg );
    break;
  case MACUSB_IOC_SET_SYNC_CFG:
    retval = device_set_sync_cfg( macdev, arg );
    break;
  case MACUSB_IOC_GET_ASYNC_CFG:
    retval = device_get_async_cfg( macdev, arg );
    break;
  case MACUSB_IOC_SET_ASYNC_CFG:
    retval = device_set_async_cfg( macdev, arg );
    break;
  case MACUSB_IOC_RQ_EOT:
    retval = device_rq_eot( macdev, arg );
    break;
  case MACUSB_IOC_RQ_N16:
    retval = device_rq_n16( macdev, arg );
    break;
  case MACUSB_IOC_RQ_N32:
    retval = device_rq_n32( macdev, arg );
    break;
  case MACUSB_IOC_RQ_ECHO:
    retval = device_rq_echo( macdev, arg );
    break;
  case MACUSB_IOC_STATUS:
    retval = device_status( macdev, arg );
    break;
  case MACUSB_IOC_STOP_RQ:
    retval = device_stop_rq( macdev, arg );
    break;
  case MACUSB_IOC_COPY_RQ:
    retval = device_copy_rq( macdev, arg );
    break;
  default:
    retval = -ENOTTY;
  }
  return retval;
}

#ifdef HAVE_COMPAT_IOCTL
static long
macusb_compat_ioctl( struct file   *file,
		     unsigned int   cmd,
		     unsigned long  arg )
{
//printk( KERN_INFO "macusb_compat_ioctl, cmd=0x%x, arg=0x%lx\n", cmd, arg );
  return macusb_ioctl( file, cmd, arg );
}
#endif

static struct file_operations macusb_fops = {
  .owner          = THIS_MODULE,
  .read           = macusb_read,
  .write          = macusb_write,
  .open           = macusb_open,
  .release        = macusb_release,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 11)
  .unlocked_ioctl = macusb_ioctl,
#else
  .ioctl          = macusb_ioctl,
#endif
#ifdef HAVE_COMPAT_IOCTL
  .compat_ioctl   = macusb_compat_ioctl,
#endif
};

/****************************************************************************/
/* sysfs calls are to be protected with kref like devfs ones */
static macusb_dev_t *
macusb_get( struct device *dev )
{
  struct usb_interface *intf = to_usb_interface( dev );
  macusb_dev_t *macdev=NULL;
  spin_lock( &macusb_lock );
  macdev = usb_get_intfdata( intf );
  if ( !macdev ) {
    spin_unlock( &macusb_lock );
    return NULL;
  }
  kref_get( &macdev->kref );
  spin_unlock( &macusb_lock );
  return macdev;
}

static void
macusb_put( macusb_dev_t *macdev )
{
  if ( macdev )
    kref_put( &macdev->kref, macusb_delete );
}

ssize_t
show_id( struct device *dev,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 13)
	 struct device_attribute *attr,
#endif
	 char *buf )
{
  macusb_dev_t *macdev=NULL;
  ssize_t rv=0;
  int did;
  if ( (macdev=macusb_get( dev )) ) {
    if ( (did=device_id( macdev )) < 0 ) {
      rv = did;
    } else {
      rv = snprintf( buf, PAGE_SIZE, "%d\n", did );
    }
  }
  macusb_put( macdev );
  return rv;
}

static DEVICE_ATTR( id, S_IRUGO, show_id, NULL );

ssize_t
show_bulk_out( struct device *dev,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 13)
	       struct device_attribute *attr,
#endif
	       char *buf )
{
  macusb_dev_t *macdev=NULL;
  ssize_t rv=0;
  if ( (macdev=macusb_get( dev )) )
  /* output format: bytes free_urbs mac_tout_urbs completed_urbs size_mismatch_urbs bad_status_urbs busy_urbs */
    rv = snprintf( buf, PAGE_SIZE, "%llu %llu %llu %llu %llu %u %u\n",
		   macdev->bulk_out.status->bytes,
		   macdev->bulk_out.status->mac_tout_urbs,
		   macdev->bulk_out.status->completed_urbs,
		   macdev->bulk_out.status->size_mismatch_urbs,
		   macdev->bulk_out.status->bad_status_urbs,
		   macdev->async_cfg.urb_num - macdev->bulk_out.status->busy_urbs,
		   macdev->bulk_out.status->busy_urbs );
  macusb_put( macdev );
  return rv;
}

static DEVICE_ATTR( bulk_out, S_IRUGO, show_bulk_out, NULL );

ssize_t
show_bulk_out_urb_occupancy( struct device *dev,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 13)
			     struct device_attribute *attr,
#endif
			     char *buf )
{
  macusb_dev_t *macdev=NULL;
  ssize_t chars=0;
  if ( (macdev=macusb_get( dev )) ) {
    int i=0;
    ssize_t rv=0;
    for ( ;i<MACUSB_URB_OCCUPANCY_HISTO_SIZE; i++ ) {
      if ( (rv=snprintf( buf+chars,
			 PAGE_SIZE-chars,
			 i < MACUSB_URB_OCCUPANCY_HISTO_SIZE-1 ? "%llu " : "%llu\n",
			 macdev->bulk_out.status->urb_occupancy[i] )) < 0 ) {
	macusb_put( macdev );
	return rv;
      }
      chars += rv;
    }
  }
  macusb_put( macdev );
  return chars;
}

static DEVICE_ATTR( bulk_out_urb_occupancy, S_IRUGO, show_bulk_out_urb_occupancy, NULL );

ssize_t
show_bulk_in( struct device *dev,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 13)
	      struct device_attribute *attr,
#endif
	      char *buf )
{
  macusb_dev_t *macdev=NULL;
  ssize_t rv=0;
  if ( (macdev=macusb_get( dev )) )
  /* output format: bytes mac_tout_urbs completed_urbs size_mismatch_urbs bad_status_urbs free_urbs busy_urbs */
    rv = snprintf( buf, PAGE_SIZE, "%llu %llu %llu %llu %llu %u %u\n",
		   macdev->bulk_in.status->bytes,
		   macdev->bulk_in.status->mac_tout_urbs,
		   macdev->bulk_in.status->completed_urbs,
		   macdev->bulk_in.status->size_mismatch_urbs,
		   macdev->bulk_in.status->bad_status_urbs,
		   macdev->async_cfg.urb_num - macdev->bulk_in.status->busy_urbs,
		   macdev->bulk_in.status->busy_urbs );
  macusb_put( macdev );
  return rv;
}

static DEVICE_ATTR( bulk_in, S_IRUGO, show_bulk_in, NULL );

ssize_t
show_bulk_in_urb_occupancy( struct device *dev,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 13)
			    struct device_attribute *attr,
#endif
			    char *buf )
{
  macusb_dev_t *macdev=NULL;
  ssize_t chars=0;
  if ( (macdev=macusb_get( dev )) ) {
    int i=0;
    ssize_t rv=0;
    for ( ;i<MACUSB_URB_OCCUPANCY_HISTO_SIZE; i++ ) {
      if ( (rv=snprintf( buf+chars,
			 PAGE_SIZE-chars,
			 i < MACUSB_URB_OCCUPANCY_HISTO_SIZE-1 ? "%llu " : "%llu\n",
			 macdev->bulk_in.status->urb_occupancy[i] )) < 0 ) {
	macusb_put( macdev );
	return rv;
      }
      chars += rv;
    }
  }
  macusb_put( macdev );
  return chars;
}

static DEVICE_ATTR( bulk_in_urb_occupancy, S_IRUGO, show_bulk_in_urb_occupancy, NULL );

/****************************************************************************/
/* 
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with devfs and the driver core
 */
static struct usb_class_driver macusb_class = {
  /*  .name       = MACUSB_NODE_PREFIX"%d", */
  .name       = NULL,
  .fops       = &macusb_fops,
  .minor_base =	MACUSB_MINOR_BASE,
};

static int
macusb_probe( struct usb_interface       *interface,
	      const struct usb_device_id *id )
{
  macusb_dev_t *macdev = NULL;
  struct usb_host_interface *iface_desc;
  struct usb_endpoint_descriptor *endpoint;
  int i;
  int retval = -ENOMEM;
  char *dev_str = NULL;

  /* allocate memory for our device state and initialize it */
  macdev = kmalloc( sizeof( *macdev ), GFP_KERNEL );
  if (macdev == NULL) {
    printk( KERN_ERR "macusb: Out of memory");
    goto error;
  }
  memset( macdev, 0, sizeof( *macdev ) );
  kref_init( &macdev->kref );

  macdev->udev = usb_get_dev( interface_to_usbdev( interface ) );
  if ( macdev->udev == NULL ) {
    printk( KERN_ERR "macusb: Failed to usb_get_dev");
    goto error;
  }
  macdev->interface = interface;
  atomic_set( &macdev->closed, 1 );
  macdev->bulk_in.process_data_locked = process_data_locked;
  macdev->status.state = MACUSB_STATE_FREE;
  rwlock_init( &macdev->rwlock );

  /* set up endpoint information */
  /* use only the first bulk-out and bulk-in endpoints */
  iface_desc = interface->cur_altsetting;
  for ( i = 0; i < iface_desc->desc.bNumEndpoints; ++i ) {

    endpoint = &iface_desc->endpoint[i].desc;
    if ( ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK) ) {
      if ( ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT) ) {
	if ( ! macdev->bulk_out.bEndpointAddr ) {
	  /* we found a bulk-out endpoint */
	  macdev->bulk_out.bEndpointAddr = endpoint->bEndpointAddress;
	  macdev->bulk_out.wMaxPacketSize = le16_to_cpu( endpoint->wMaxPacketSize );
	}
      } else if ( ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) ) {
	if ( ! macdev->bulk_in.bEndpointAddr ) {
	  /* we found a bulk-in endpoint */
	  macdev->bulk_in.bEndpointAddr = endpoint->bEndpointAddress;
	  macdev->bulk_in.wMaxPacketSize = le16_to_cpu( endpoint->wMaxPacketSize );
	}
      }
    }
  }
  if ( ! macdev->bulk_out.bEndpointAddr ) {
    printk( KERN_ERR "macusb: Could not find bulk-out endpoints");
    retval = -EIO;
    goto error;
  }
  if ( ! macdev->bulk_in.bEndpointAddr ) {
    printk( KERN_ERR "macusb: Could not find bulk-in endpoints");
    retval = -EIO;
    goto error;
  }
  if ( (retval=set_sync_cfg( macdev, &macusb_dev_sync_default )) ) {
    printk( KERN_ERR "macusb: Could not set sync configuration");
    goto error;
  }
  if ( (retval=set_async_cfg( macdev, &macusb_dev_async_default )) ) {
    printk( KERN_ERR "macusb: Could not set async configuration");
    goto error;
  }
  if ( macdev->udev->speed != USB_SPEED_HIGH ) {
    /* low speed and full speed devices are not acceptable
    macdev->async_cfg.mac_tout = 0;
    printk( KERN_WARNING "macusb: %s: *WARNING* Set mac_tout to zero cause device speed is not high (%i)", macdev->name, macdev->udev->speed );
    */
    retval = -EIO;
    printk( KERN_ERR "macusb: Failed to probe, device speed is not high (%i)", macdev->udev->speed );
    goto error;
  }
  /* save our data pointer in this interface device */
  spin_lock( &macusb_lock );
  usb_set_intfdata( interface, macdev );
  spin_unlock( &macusb_lock );

  /* register the device */
  if ( macdev->udev->descriptor.idProduct == MACUSB_PRODUCT_ID_LE99S ) {
    dev_str = MACUSB_LE99S_STR;
  } else if ( macdev->udev->descriptor.idProduct  == MACUSB_PRODUCT_ID_LE98 ) {
    dev_str = MACUSB_LE98_STR;
  } else if ( macdev->udev->descriptor.idProduct  == MACUSB_PRODUCT_ID_EM11 ) {
    dev_str = MACUSB_EM11_STR;
  } else {
    printk( KERN_ERR "macusb: 0x%x: Unknown idProduct.", macdev->udev->descriptor.idProduct );
    goto error;
  }
  snprintf( macdev->name,
	    MACUSB_NAME_MAX_SIZE,
	    "%s_%s_%d-%%d",
	    MACUSB_NODE_PREFIX,
	    dev_str,
	    macdev->udev->descriptor.bcdDevice );
  macusb_class.name = macdev->name;
  if ( (retval=usb_register_dev( interface, &macusb_class )) ) {
    /* something prevented us from registering this driver */
    printk( KERN_ERR "macusb: Not able to get a minor for this device." );
    goto error;
  }
  snprintf( macdev->bulk_in.name,
	    MACUSB_NAME_MAX_SIZE,
	    "%s_%s_%s_bulk-in",
	    MACUSB_NODE_PREFIX,
	    dev_str,
	    kobject_name( &interface->dev.kobj ) );

  if ( (retval=device_create_file( &interface->dev, &dev_attr_id )) ) {
    printk( KERN_ERR "macusb: failed to create attribute 'id': %i\n", retval );
    goto error;
  }
  if ( (retval=device_create_file( &interface->dev, &dev_attr_bulk_out )) ) {
    printk( KERN_ERR "macusb: failed to create attribute 'bulk_out': %i\n", retval );
    goto error;
  }
  if ( (retval=device_create_file( &interface->dev, &dev_attr_bulk_out_urb_occupancy )) ) {
    printk( KERN_ERR "macusb: failed to create attribute 'bulk_out_urb_occupancy': %i\n", retval );
    goto error;
  }
  if ( (retval=device_create_file( &interface->dev, &dev_attr_bulk_in )) ) {
    printk( KERN_ERR "macusb: failed to create attribute 'bulk_in': %i\n", retval );
    goto error;
  }
  if ( (retval=device_create_file( &interface->dev, &dev_attr_bulk_in_urb_occupancy )) ) {
    printk( KERN_ERR "macusb: failed to create attribute 'bulk_in_urb_occupancy': %i\n", retval );
    goto error;
  }
  /* init has finished successfully, blare about it */
  printk( KERN_INFO "macusb: device '%s' now attached to minor #%d\n", macdev->name, interface->minor );
  return 0;

error:
  if ( macdev ) {
    device_remove_file( &interface->dev, &dev_attr_id );
    device_remove_file( &interface->dev, &dev_attr_bulk_out );
    device_remove_file( &interface->dev, &dev_attr_bulk_out_urb_occupancy );
    device_remove_file( &interface->dev, &dev_attr_bulk_in );
    device_remove_file( &interface->dev, &dev_attr_bulk_in_urb_occupancy );

    spin_lock( &macusb_lock );
    usb_set_intfdata( interface, NULL );
    spin_unlock( &macusb_lock );

    usb_deregister_dev( interface, &macusb_class );

    kref_put( &macdev->kref, macusb_delete );
  }
  return retval;
}

static void
macusb_disconnect( struct usb_interface *interface )
{
  macusb_dev_t *macdev;
  int minor = interface->minor;

  device_remove_file( &interface->dev, &dev_attr_id );
  device_remove_file( &interface->dev, &dev_attr_bulk_out );
  device_remove_file( &interface->dev, &dev_attr_bulk_out_urb_occupancy );
  device_remove_file( &interface->dev, &dev_attr_bulk_in );
  device_remove_file( &interface->dev, &dev_attr_bulk_in_urb_occupancy );

  spin_lock( &macusb_lock );
  macdev = usb_get_intfdata( interface );
  usb_set_intfdata( interface, NULL );
  spin_unlock( &macusb_lock );

  /* give back our minor */
  usb_deregister_dev( interface, &macusb_class );

  printk( KERN_INFO "macusb: device '%s', minor #%d now disconnected\n", macdev->name, minor );

  /* decrement our usage count */
  kref_put( &macdev->kref, macusb_delete );
}

/****************************************************************************/

static struct usb_driver macusb_driver = {
  .name       = "macusb",
  .probe      = macusb_probe,
  .disconnect =	macusb_disconnect,
  .id_table   = macusb_table,
};

static int __init
macusb_init( void )
{
  int rv=0;

  printk( KERN_INFO "macusb: v" MACUSB_VERSION_STR " by Alexey Filin, compiled at " __DATE__ "\n" );

  /* register this driver with the USB subsystem */
  if ( (rv=usb_register( &macusb_driver )) ) {
    printk( KERN_ERR "macusb: usb_register failed. Error number %d\n", rv );

  } else {
    if ( !(bottom_half_workqueue=create_workqueue( "macusb" )) ) {
      printk( KERN_ERR "macusb: failed to create workqueue\n" );
      rv = EPERM;

    } else {
      /* set sync default values */
      macusb_dev_sync_default.tx_tout = MACUSB_SYNC_TXTOUT;
      macusb_dev_sync_default.rx_tout = MACUSB_SYNC_RXTOUT;
      /* set async default values */
      if ( MACUSB_ASYNC_URBNUM < MACUSB_ASYNC_MAX_URBNUM )
	macusb_dev_async_default.urb_num = MACUSB_ASYNC_URBNUM;
      if ( MACUSB_ASYNC_MAC_TOUT < MACUSB_ASYNC_MAX_MAC_TOUT )
	macusb_dev_async_default.mac_tout = MACUSB_ASYNC_MAC_TOUT;
    }
  }
  return rv;
}

static void __exit
macusb_exit( void )
{
  flush_workqueue( bottom_half_workqueue );
  destroy_workqueue( bottom_half_workqueue );

  /* deregister this driver with the USB subsystem */
  usb_deregister( &macusb_driver );
}

module_init( macusb_init );
module_exit( macusb_exit );
