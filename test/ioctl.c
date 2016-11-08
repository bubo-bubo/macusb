#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <popt.h>
#include <sys/ioctl.h>

#include "macusb.h"

#define NAME_BUFF_SIZE 256

int cmd = 0;
char *opt_str_arg = NULL;
macusb_sync_cfg_t scfg={ 0, 0 };
macusb_async_cfg_t acfg={ 0, 0, 0 };
char macusb_node_path[NAME_BUFF_SIZE];

int
get_unsigned( const char   *str,
	      unsigned int *arr,
	      unsigned int  arr_size )
{
  const char *beg=str;
  char *end=NULL;
  unsigned int n;
  int num=0;
  for (;;) {
    n = strtoul( beg, &end, 10 );
    if ( end == beg )
      return -1;
    arr[num++] = n;
    if ( *end == '\0' )
      break;
    if ( *end == ':' )
      end++;
    if ( num == arr_size )
      return -2;
    beg = end;
  }
  return num;
}

int
process_options( int argc, const char* argv[] )
{
  char c;
  char *opt_str_arg = NULL;
  poptContext optCon;
  int rv=0;
  unsigned int args[4];
  int num=0;

  struct poptOption optionsTable[] = {
    { "path", 'p', POPT_ARG_STRING, &opt_str_arg, 'p', "set node path", "PATH" },
    { "sync-cfg", '1', POPT_ARG_STRING | POPT_ARGFLAG_OPTIONAL, &opt_str_arg, '1', "get/set configuration for synchronous mode", "[TXTOUT:RXTOUT]" },
    { "async-cfg", '2', POPT_ARG_STRING | POPT_ARGFLAG_OPTIONAL, &opt_str_arg, '2', "get/set configuration for asynchronous mode", "[URBNUM:MACTOUT:URBTOUT]" },
    { "status", 's', 0, 0, 's', "get device node status", 0 },
    { "stop", 't', 0, 0, 't', "stop request", 0 },
    { "reset-usb", 'r', 0, 0, 'r', "reset USB device", 0 },
    { "reset-ctl", 'z', 0, 0, 'z', "reset bus controller", 0 },
    { "clear", 'c', 0, 0, 'c', "clear driver buffers and reset counters", 0 },
    
    { "usb-id", 'i', 0, 0, 'i', "get USB device id", 0 },
     POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0 }
  };

  snprintf( macusb_node_path, NAME_BUFF_SIZE, "/dev/macusb_le98_9-0" );

  optCon = poptGetContext( NULL, argc, argv, optionsTable, 0 );

  while ( (c = poptGetNextOpt( optCon )) >= 0 ) {
    switch ( c ) {
    case 'p':
      snprintf( macusb_node_path, NAME_BUFF_SIZE, "%s", opt_str_arg );
      break;
    case '1':
      if ( !opt_str_arg ) {
	cmd = MACUSB_IOC_GET_SYNC_CFG;

      } else {
	cmd = MACUSB_IOC_SET_SYNC_CFG;
	if ( (num=get_unsigned( opt_str_arg, args, 2 )) < 0 || num != 2 ) {
	  fprintf( stderr, "%s: wrong sync-cfg argument\n", opt_str_arg );
	  rv = 1;
	  break;
	}
	scfg.tx_tout = args[0];
	scfg.rx_tout = args[1];
      }
      break;
    case '2':
      if ( !opt_str_arg ) {
	cmd = MACUSB_IOC_GET_ASYNC_CFG;

      } else {
	cmd = MACUSB_IOC_SET_ASYNC_CFG;
	if ( (num=get_unsigned( opt_str_arg, args, 3 )) < 0 || num != 3 ) {
	  fprintf( stderr, "%s: wrong async-cfg argument\n", opt_str_arg );
	  rv = 2;
	  break;
	}
	acfg.urb_num = args[0];
	acfg.mac_tout = args[1];
	acfg.urb_tout = args[2];
      }
      break;
    case 's':
      cmd = MACUSB_IOC_STATUS;
      break;
    case 't':
      cmd = MACUSB_IOC_STOP_RQ;
      break;
    case 'r':
      cmd = MACUSB_IOC_RESET_DEVICE;
      break;
    case 'z':
      cmd = MACUSB_IOC_RESET_CTL;
      break;
    case 'c':
      cmd = MACUSB_IOC_CLEAR;
      break;
    case 'i':
      cmd = MACUSB_IOC_DEVICE_ID;
      break;
    default:;
    }
  }
  if ( c < -1 ) {
    fprintf( stderr, "%s: %s\n",
	     poptBadOption( optCon, POPT_BADOPTION_NOALIAS ),
	     poptStrerror( c ) );
    rv = 3;
  } else if ( !cmd || poptGetArg( optCon ) ) {
    poptPrintHelp( optCon, stdout, 0 );
    rv = 4;
  }
  poptFreeContext( optCon );
  return rv;
}

void
print_status( macusb_status_t *st )
{
  int i=0;
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
  printf( "state                  %u (%s)\n", st->state, st->state < MACUSB_STATE_END+1 ? statestr[st->state] : "" );
  printf( "err                    %i (%s)\n", st->err, strerror( -st->err ) );
  printf( "  TX stat:\n" );
  printf( "    bytes              %lu\n", st->bulk_out.bytes );
  printf( "    mac_tout_urbs      %lu\n", st->bulk_out.mac_tout_urbs );
  printf( "    completed_urbs     %lu\n", st->bulk_out.completed_urbs );
  printf( "    size_mismatch_urbs %lu\n", st->bulk_out.size_mismatch_urbs );
  printf( "    bad_status_urbs    %lu\n", st->bulk_out.bad_status_urbs );
  printf( "    busy_urbs          %u (currently)\n", st->bulk_out.busy_urbs );
  printf( "    urb_occupancy      " );
  for ( i=0; i<MACUSB_URB_OCCUPANCY_HISTO_SIZE; i++ )
    printf( ( i < MACUSB_URB_OCCUPANCY_HISTO_SIZE - 1 ) ? "%lu " : "%lu\n", st->bulk_out.urb_occupancy[i] );
  printf( "  RX stat:\n" );
  printf( "    bytes              %lu\n", st->bulk_in.bytes );
  printf( "    mac_tout_urbs      %lu\n", st->bulk_in.mac_tout_urbs );
  printf( "    completed_urbs     %lu\n", st->bulk_in.completed_urbs );
  printf( "    size_mismatch_urbs %lu\n", st->bulk_in.size_mismatch_urbs );
  printf( "    bad_status_urbs    %lu\n", st->bulk_in.bad_status_urbs );
  printf( "    busy_urbs          %u (currently)\n", st->bulk_in.busy_urbs );
  printf( "    urb_occupancy      " );
  for ( i=0; i<MACUSB_URB_OCCUPANCY_HISTO_SIZE; i++ )
    printf( ( i < MACUSB_URB_OCCUPANCY_HISTO_SIZE - 1 ) ? "%lu " : "%lu\n", st->bulk_in.urb_occupancy[i] );
}

int
main( int argc, const char* argv[] )
{
  int fd=-1, rv=0;
  macusb_status_t st;
  errno=0;

  if ( (rv=process_options( argc, argv )) )
    exit( rv );

  if ( (fd=open( macusb_node_path, O_RDONLY )) == -1 ) {
    rv = errno;
    perror( macusb_node_path );
    goto exit;

  } else {
    printf( "node path: %s\n", macusb_node_path );
  }
  switch ( cmd ) {
  case MACUSB_IOC_GET_SYNC_CFG:
    if ( (rv=ioctl( fd, cmd, &scfg )) ) {
      perror("MACUSB_IOC_GET_SYNC_CFG");
    } else {
      printf( "GET_SYNC_CFG:\n tx_tout=%u\n rx_tout=%u\n",
	      scfg.tx_tout, scfg.rx_tout );
    }
    break;
  case MACUSB_IOC_SET_SYNC_CFG:
    printf( "SET_SYNC_CFG:\n tx_tout=%u\n rx_tout=%u\n",
	    scfg.tx_tout, scfg.rx_tout );
    if ( (rv=ioctl( fd, cmd, &scfg )) ) {
      perror("MACUSB_IOC_SET_SYNC_CFG");
    }
    break;
  case MACUSB_IOC_GET_ASYNC_CFG:
    if ( (rv=ioctl( fd, cmd, &acfg )) ) {
      perror("MACUSB_IOC_GET_ASYNC_CFG");
    } else {
      printf( "GET_ASYNC_CFG:\n urb_num=%u\n mac_tout=%u\n urb_tout=%u\n",
	      acfg.urb_num, acfg.mac_tout, acfg.urb_tout );
    }
    break;
  case MACUSB_IOC_SET_ASYNC_CFG:
    printf( "SET_ASYNC_CFG:\n urb_num=%u\n mac_tout=%u\n urb_tout=%u\n",
	    acfg.urb_num, acfg.mac_tout, acfg.urb_tout );
    if ((rv=ioctl( fd, cmd, &acfg )) ) {
      perror("MACUSB_IOC_SET_ASYNC_CFG");
    }
    break;
  case MACUSB_IOC_STATUS:
    memset( &st, 0, sizeof( st ) );
    if ( (rv=ioctl( fd, cmd, &st )) < 0 ) {
      perror("MACUSB_IOC_STATUS");
    } else {
      print_status( &st );
    }
    break;
  case MACUSB_IOC_STOP_RQ:
    memset( &st, 0, sizeof( st ) );
    if ( (rv=ioctl( fd, cmd, &st )) == -1 ) {
      perror("MACUSB_IOC_RQ_STOP");
    } else {
      print_status( &st );
    }
    break;
  case MACUSB_IOC_RESET_DEVICE:
    if ( (rv=ioctl( fd, cmd )) == -1 ) {
      perror("MACUSB_IOC_RESET_DEVICE");
    }
    break;
  case MACUSB_IOC_RESET_CTL:
    if ( (rv=ioctl( fd, cmd )) == -1 ) {
      perror("MACUSB_IOC_RESET_CTL");
    }
    break;
  case MACUSB_IOC_CLEAR:
    if ( (rv=ioctl( fd, cmd )) == -1 ) {
      perror("MACUSB_IOC_CLEAR");
    }
    break;
  case MACUSB_IOC_DEVICE_ID:
    if ( (rv=ioctl( fd, cmd )) < 0 ) {
      perror("MACUSB_IOC_DEVICE_ID");
    } else {
      printf( "id=%i\n", rv );
    }
    break;
  default:;
  }
  if ( close( fd ) ) {
    rv = errno;
    perror( "close" );
  }

exit:
  return rv;
}
