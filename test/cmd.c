#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <popt.h>

#include "macusb.h"
#include "bcu/bcu_em11.h"
#include "bcu/bcu_le98.h"
#include "bcu/bcu_dev.h"
#include "bcu/bcu_reg.h"

//#define IDLE_RUN 1

int node_id=9;
char *opt_str_arg=NULL;
int opt_int_arg=-1;
int verbosity_level=1;
char *bcu_type=NULL;

union LE98StatusReg {
  struct {
    u_int16_t T1        : 1;
    u_int16_t T2        : 1;
    u_int16_t T3        : 1;
    u_int16_t Ready     : 1;
    u_int16_t Tr        : 1;
    u_int16_t Err       : 1;
    u_int16_t SNR       : 1;
    u_int16_t SDR       : 1;
    u_int16_t Auto      : 1;
    u_int16_t Sy1       : 1;
    u_int16_t Sy2       : 1;
    u_int16_t FSNRReady : 1;
    u_int16_t dummy     : 3;
    u_int16_t DataFlag  : 1;
  } bits;
  u_int16_t buf;
};

union LE98CommandReg {
  struct {
    u_int16_t A     : 6;
    u_int16_t N     : 6;
    u_int16_t F     : 3;
    u_int16_t reset : 1;
  } bits;
  u_int16_t buf;
};

union LE98AddressReg {
  struct {
    u_int16_t A     : 6;
    u_int16_t N     : 6;
    u_int16_t dummy : 3;
    u_int16_t error : 1;
  } bits;
  u_int16_t buf;
};

struct poptOption optionsTable[] = {
  { "addresses", 'a', 0, 0, 'a', "MISS module addresses (sequential number read, PChN)", 0 },
  { "cnaf", 'c', POPT_ARG_STRING, &opt_str_arg, '2', "run control NAF", "N:A:F" },
  { "data", 'd', 0, 0, 'd', "MISS module data (sequential data read, PChI)", 0 },
  { "node", 'n', POPT_ARG_INT, &node_id, 'n', "set device node id. Position independent option", "ID" },
  { "init", 'i', 0, 0, 'i', "initialise MISS controller", 0 },
  { "mode", 'm', POPT_ARG_STRING | POPT_ARGFLAG_OPTIONAL, &opt_str_arg, 'm', "get/set MISS mode", "MODE" },
  { "oc", 'o', 0, 0, 'o', "reset MISS crate (OC)", 0 },
  { "rnaf", 'r', POPT_ARG_STRING, &opt_str_arg, 'r', "read and print data with MISS NAF", "N:A:F" },
  { "state", 's', 0, 0, 's', "show device node state", 0 },
  { "type", 't', POPT_ARG_STRING, &bcu_type, 't', "set BCU type", "TYPE" },
  { "wnaf", 'w', POPT_ARG_STRING, &opt_str_arg, 'w', "write data D with MISS NAF", "N:A:F:D" },
  { "read-reg", 'x', POPT_ARG_INT, &opt_int_arg, 'x', "read register by address A", "ADDR" },
  { "write-reg", 'y', POPT_ARG_STRING, &opt_str_arg, 'y', "write data D to MISS controller register by address A", "A:D" },
  { "cmd-num", 'b', POPT_ARG_INT, &opt_int_arg, 'b', "repeat command NUM times in one request", "NUM" },
  { "rq-num", 'q', POPT_ARG_INT, &opt_int_arg, 'q', "run request NUM times", "NUM" },
  { "reset-usb", 'u', 0, 0, 'u', "reset USB device", 0 },
  { "reset-ctl", 'z', 0, 0, 'z', "reset bus controller", 0 },
  { "clear", '5', 0, 0, '5', "clear driver buffers and reset counters", 0 },
  { "verbosity", 'v', POPT_ARG_INT, &verbosity_level, 'v', "set verbosity LEVEL: 0 -- silent, 1 -- print got data (default), 2 -- print got data and TX/RX traffic. Position independent option", "NUM" },
  POPT_AUTOHELP
  { NULL, 0, 0, NULL, 0 }
};

/* set device id, verbosity, print help */
int
preprocess_options( int argc, const char* argv[] )
{
  int rv=0;
  char c;
  poptContext optCon;
  optCon = poptGetContext( NULL, argc, argv, optionsTable, 0 );

  if ( argc < 2 ) {
    poptPrintHelp( optCon, stderr, 0 );
    return 1;
  }
  while ( (c = poptGetNextOpt( optCon )) >= 0 ) {
    switch ( c ) {
    case 'n':
      if ( node_id < 0 ) {
	fprintf( stderr, "%i: wrong device node id\n", node_id );
	rv = 2;
      }
      break;
    case 't':
      if ( strncmp( "LE98", bcu_type, strlen( "LE98" ) ) &&
	   strncmp( "EM11", bcu_type, strlen( "EM11" ) ) ) {
	fprintf( stderr, "%s: wrong BCU type\n", bcu_type );
	rv = 3;
      }
      break;
    case 'v':
      if ( verbosity_level < 0 || verbosity_level > 2 ) {
	fprintf( stderr, "%i: wrong verbosity level\n", verbosity_level );
	rv = 4;
      }
      break;
    }
    if ( rv )
      break;
  }
  if ( poptGetArg( optCon ) ) {
    poptPrintHelp( optCon, stdout, 0 );
    rv = 5;
  }
  if ( c < -1 ) {
    fprintf( stderr, "%s: %s\n",
	     poptBadOption( optCon, POPT_BADOPTION_NOALIAS ),
	     poptStrerror( c ) );
    rv = 6;
  }
  poptFreeContext( optCon );
  return rv;
}

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

/* device with attributes */
typedef struct bcu_device {
  bcu_dev_t       dev;
  macusb_buffer_t data;
  macusb_status_t status;
} bcu_device_t;

void
print_error( bcu_device_t *bcu,
	     const char   *desc,
	     int           errors )
{
  if ( errors < 0 ) {
    perror( bcu->dev.path );

  } else if ( errors > 0 ) {
    if ( errors & BCU_CTL_ERR_MASK ) {
      fprintf( stderr, "0x%x: %s failed\n\n", errors, desc );
    }
    if ( errors & BCU_ENC_ERR_MASK ) {
      fprintf( stderr, "0x%x: failed to encode %s\n\n", errors, desc );
    }
    if ( errors & BCU_DEC_ERR_MASK ) {
      fprintf( stderr, "0x%x: failed to decode %s\n\n", errors, desc );
    }
  }
}

int
exec_buf( bcu_device_t *bcu,
	  const char   *desc )
{
  int rv=0;
  u_int16_t *ptr=0;
  unsigned int i=0;
  if ( verbosity_level >= 2 ) {
    ptr=(u_int16_t *)bcu->data.tx_buf;
    printf( "TX %u bytes:\n", bcu->data.tx_bytes );
    for ( i=0; i<bcu->data.tx_bytes/2; i++, ptr++ ) {
      printf( "  %03u: 0x%x\n", i, *ptr );
    }
  }
#ifndef IDLE_RUN
  if ( (rv=bcu_dev_exec_buf( &bcu->dev, &bcu->data )) ) {
    print_error( bcu, desc, rv );
    return rv;
  }
#endif
  if ( verbosity_level >= 2 ) {
    printf( "RX %u bytes:\n", bcu->data.rx_bytes );
  }
  ptr=(u_int16_t *)bcu->data.rx_buf;
  if ( verbosity_level >= 2 ) {
    for ( i=0; i<bcu->data.rx_bytes/2; i++, ptr++ ) {
      printf( "  %03u: 0x%x\n", i, *ptr );
    }
  }
  return 0;
}

u_int16_t
naf( u_int16_t N, u_int16_t A, u_int16_t F, u_int16_t reset )
{
  union LE98CommandReg reg = {
    .bits.A = A,
    .bits.N = N,
    .bits.F = F,
    .bits.reset = reset,
  };
  return reg.buf;
}

int
snr( bcu_device_t *bcu )
{
  int rv=0;
  union LE98StatusReg state;
  union LE98AddressReg addr;
  u_int16_t data;
#ifdef IDLE_RUN
  return 0;
#endif
  state.buf=0;
  addr.buf=0;
  data=0;
  if ( (rv=bcu_dev_exec_cmd( &bcu->dev, BCU_DEV_CMD_SNR_START, &state.buf, &addr.buf, &data )) ) {
    print_error( bcu, "BCU_DEV_CMD_SNR_START", rv );
    return rv;
  }

  while ( !rv && state.bits.DataFlag ) {
    state.buf=0;
    addr.buf=0;
    data=0;
    if ( (rv=bcu_dev_exec_cmd( &bcu->dev, BCU_DEV_CMD_SNR_READ, &state.buf, &addr.buf, &data )) ) {
      print_error( bcu, "BCU_DEV_CMD_SNR_READ", rv );

    } else {
      if ( verbosity_level >= 1 )
	printf( "0x%04x: N=%2u A=%2u\n", addr.buf, addr.bits.N, addr.bits.A );
    }
  }

  state.buf=0;
  addr.buf=0;
  data=0;
  if ( (rv=bcu_dev_exec_cmd( &bcu->dev, BCU_DEV_CMD_SNR_STOP, &state.buf, &addr.buf, &data )) ) {
    print_error( bcu, "BCU_DEV_CMD_SNR_STOP", rv );
  }
  return rv;
}

int
sdr( bcu_device_t *bcu )
{
  int rv=0;
  union LE98StatusReg state;
  union LE98AddressReg addr;
  u_int16_t data;
#ifdef IDLE_RUN
  return 0;
#endif
  state.buf=0;
  addr.buf=0;
  data=0;
  if ( (rv=bcu_dev_exec_cmd( &bcu->dev, BCU_DEV_CMD_SDR_START, &state.buf, &addr.buf, &data )) ) {
    print_error( bcu, "BCU_DEV_CMD_SDR_START", rv );
    return rv;
  }

  while ( !rv && state.bits.DataFlag ) {
    state.buf=0;
    addr.buf=0;
    data=0;
    if ( (rv=bcu_dev_exec_cmd( &bcu->dev, BCU_DEV_CMD_SDR_READ, &state.buf, &addr.buf, &data )) ) {
      print_error( bcu, "BCU_DEV_CMD_SDR_READ", rv );

    } else {
      if ( verbosity_level >= 1 )
	printf( "0x%04x: N=%2u A=%2u, 0x%04x: D=%4u\n", addr.buf, addr.bits.N, addr.bits.A, data, data );
    }
  }

  state.buf=0;
  addr.buf=0;
  data=0;
  if ( (rv=bcu_dev_exec_cmd( &bcu->dev, BCU_DEV_CMD_SDR_STOP, &state.buf, &addr.buf, &data )) ) {
    print_error( bcu, "BCU_DEV_CMD_SDR_STOP", rv );
  }
  return rv;
}

int
main( int argc, const char *argv[] )
{
  int rv=0;
  int i=0, j=0;
  bcu_dev_cmd_t cmd;
  bcu_device_t bcu;
  union LE98AddressReg addr;
  u_int16_t data;
  u_int16_t tmp;
  char *desc="";
  int print_result=0;
  char c;
  poptContext optCon;
  unsigned int args[4];
  int num=0;
  unsigned int rq_num = 1; /* send request rq_num times */
  int cmd_num = 1; /* repeat command NUM times in one request */
  int finish=0;

  if ( (rv=preprocess_options( argc, argv )) )
    exit( rv );
  
  if ( bcu_type == NULL ||
       !strncmp( "LE98", bcu_type, strlen( "LE98" ) ) )
    bcu_le98_init( &bcu.dev );
  else if ( strncmp( "EM11", bcu_type, strlen( "EM11" ) ) )
    bcu_em11_init( &bcu.dev );

  if ( strlen( argv[0] ) >= strlen( "cmd_sync" ) &&
       !strncmp( argv[0]+strlen( argv[0] )-strlen( "cmd_sync" ),
		 "cmd_sync",
		 strlen( "cmd_sync" ) ) ) {
    bcu.dev.transfer_flags = BCU_DEV_FLAG_SYNC;
  }

#ifndef IDLE_RUN
  if ( (rv=bcu_dev_open_id( &bcu.dev, node_id )) ) {
    if ( errno )
      perror( bcu.dev.path );
    else
      fprintf( stderr, "%i: failed to open device ID", node_id );
    goto exit;
  }
#endif
  optCon = poptGetContext( NULL, argc, argv, optionsTable, 0 );

  while ( !finish && (c=poptGetNextOpt( optCon )) >= 0 ) {
    print_result = 0;
    switch ( c ) {
    case 'a':
      if ( (rv=snr( &bcu )) )
	break;
      continue;
    case 'c':
      if ( (num=get_unsigned( opt_str_arg, args, 3 )) < 0 || num != 3 ) {
	fprintf( stderr, "%s: wrong control NAF arguments\n", opt_str_arg );
	rv = 1;
	break;
      }
      desc = "control NAF";
      cmd = BCU_DEV_CMD_NAF_CONTROL;
      addr.buf = naf( args[0], args[1], args[2], 0 );
      break;
    case 'd':
      if ( (rv=sdr( &bcu )) )
	break;
      continue;
    case 'i':
      desc = "init BCU";
      cmd = BCU_DEV_CMD_INIT_CTL;
      break;
    case 'm':
      if ( !opt_str_arg ) {
	desc = "get mode";
	cmd = BCU_DEV_CMD_MODE_GET;

      } else {
	if ( (num=get_unsigned( opt_str_arg, args, 1 )) < 0 || num != 1 ) {
	  fprintf( stderr, "%s: wrong MISS mode arguments\n", opt_str_arg );
	  rv = 1;
	  break;
	}
	if ( args[0] < 0 || args[0] > 7 ) {
	  fprintf( stderr, "%s: wrong MISS mode arguments\n", opt_str_arg );
	  rv = 1;
	  break;

	} else {
	  desc = "set mode";
	  cmd = BCU_DEV_CMD_MODE_SET;
	  data = (args[0] << 6);
	}
      }
      break;
    case 'o':
      desc = "reset MISS";
      cmd = BCU_DEV_CMD_NAF_CONTROL;
      addr.buf = naf( 0, 0, 0, 1 );
      break;
    case 'r':
      if ( (num=get_unsigned( opt_str_arg, args, 3 )) < 0 || num != 3 ) {
	fprintf( stderr, "%s: wrong read NAF arguments\n", opt_str_arg );
	rv = 1;
	break;
      }
      desc = "read NAF";
      cmd = BCU_DEV_CMD_NAF_READ;
      addr.buf = naf( args[0], args[1], args[2], 0 );
      print_result = 1;
      break;
    case 's':
      memset( &bcu.status, 0, sizeof( bcu.status ) );
      if ( (rv=bcu_dev_status( &bcu.dev, &bcu.status )) ) {
	print_error( &bcu, "BCU status", rv );
	break;
      }
      printf( "state                  %u (%s)\n", bcu.status.state, bcu_dev_state_str( bcu.status.state ) );
      printf( "err                    %i (%s)\n", bcu.status.err, strerror( -bcu.status.err ) );
      printf( "  TX stat:\n" );
      printf( "    bytes              %lu\n", bcu.status.bulk_out.bytes );
      printf( "    mac_tout_urbs      %lu\n", bcu.status.bulk_out.mac_tout_urbs );
      printf( "    completed_urbs     %lu\n", bcu.status.bulk_out.completed_urbs );
      printf( "    size_mismatch_urbs %lu\n", bcu.status.bulk_out.size_mismatch_urbs );
      printf( "    bad_status_urbs    %lu\n", bcu.status.bulk_out.bad_status_urbs );
      printf( "    busy_urbs          %u (currently)\n", bcu.status.bulk_out.busy_urbs );
      printf( "    urb_occupancy      " );
      for ( i=0; i<MACUSB_URB_OCCUPANCY_HISTO_SIZE; i++ )
	printf( ( i < MACUSB_URB_OCCUPANCY_HISTO_SIZE - 1 ) ? "%lu " : "%lu\n", bcu.status.bulk_out.urb_occupancy[i] );
      printf( "  RX stat:\n" );
      printf( "    bytes              %lu\n", bcu.status.bulk_in.bytes );
      printf( "    mac_tout_urbs      %lu\n", bcu.status.bulk_in.mac_tout_urbs );
      printf( "    completed_urbs     %lu\n", bcu.status.bulk_in.completed_urbs );
      printf( "    size_mismatch_urbs %lu\n", bcu.status.bulk_in.size_mismatch_urbs );
      printf( "    bad_status_urbs    %lu\n", bcu.status.bulk_in.bad_status_urbs );
      printf( "    busy_urbs          %u (currently)\n", bcu.status.bulk_in.busy_urbs );
      printf( "    urb_occupancy      " );
      for ( i=0; i<MACUSB_URB_OCCUPANCY_HISTO_SIZE; i++ )
	printf( ( i < MACUSB_URB_OCCUPANCY_HISTO_SIZE - 1 ) ? "%lu " : "%lu\n", bcu.status.bulk_in.urb_occupancy[i] );
      continue;
    case 'w':
      if ( (num=get_unsigned( opt_str_arg, args, 4 )) < 0 || num != 4 ) {
	fprintf( stderr, "%s: wrong write NAF arguments\n", opt_str_arg );
	rv = 1;
	break;
      }
      desc = "write NAF";
      cmd = BCU_DEV_CMD_NAF_WRITE;
      addr.buf = naf( args[0], args[1], args[2], 0 );
      data = args[3];
      break;
    case 'x':
      if ( opt_int_arg < 0 ) {
	fprintf( stderr, "%i: wrong register address\n\n", opt_int_arg );
	rv = 1;
	break;
      }
      desc = "read register";
      cmd = BCU_DEV_CMD_REG_READ;
      addr.buf = opt_int_arg;
      print_result = 1;
      break;
    case 'y':
      if ( (num=get_unsigned( opt_str_arg, args, 2 )) < 0 || num != 2 ) {
	fprintf( stderr, "%s: wrong wrong write register arguments\n", opt_str_arg );
	rv = 1;
	break;
      }
      desc = "write register";
      cmd = BCU_DEV_CMD_REG_WRITE;
      addr.buf = args[0];
      data = args[1];
      break;
    case 'b':
      if ( opt_int_arg < 0 ) {
	fprintf( stderr, "%i: wrong cmd-num\n\n", opt_int_arg );
	rv = 1;
	break;
      }
      cmd_num = opt_int_arg;
      continue;
    case 'q':
      if ( opt_int_arg < 0 ) {
	fprintf( stderr, "%i: wrong rq-num\n\n", opt_int_arg );
	rv = 1;
	break;
      }
      rq_num = opt_int_arg;
      continue;
    case 'u':
      if ( (rv=bcu_dev_reset( &bcu.dev )) < 0 ) {
	perror( bcu.dev.path );
      }
      finish = 1;
      continue;
    case 'z':
      if ( (rv=bcu_dev_reset_ctl( &bcu.dev )) < 0 ) {
	perror( bcu.dev.path );
      }
      finish = 1;
      continue;
    case '5':
      if ( (rv=bcu_dev_clear( &bcu.dev )) < 0 ) {
	perror( bcu.dev.path );
      }
      finish = 1;
      continue;
    default:
      continue; /* parsed in preprocess_options() */
    }
    if ( rv )
      break;
    if ( cmd_num && rq_num ) {
      memset( &bcu.data, 0, sizeof( bcu.data ) );
      for ( j=0; j<cmd_num; j++ ) {
	if ( (rv=bcu_dev_encode_cmd( &bcu, &bcu.data, cmd, addr.buf, data )) ) {
	  break;
	}
      }
      for ( i=0; i<rq_num; i++ ) {
	unsigned int offset = 0;
	bcu.data.rx_bytes = 0;
	if ( (rv=exec_buf( &bcu, desc )) ) {
	  printf( "iteration %i failed\n", i );
	  break;
	}
#ifndef IDLE_RUN
	for ( j=0; j<cmd_num; j++ ) {
	  addr.buf=0;
	  data=0;
	  if ( (rv=bcu_dev_decode_cmd( &bcu, &bcu.data, &offset, cmd, &tmp, &addr.buf, &data )) ) {
	    print_error( &bcu, desc, rv );
	    break;
	  }
	  if ( verbosity_level >= 1 && print_result )
	    printf( "0x%04x: N=%2u A=%2u, 0x%04x: D=%4u\n", addr.buf, addr.bits.N, addr.bits.A, data, data );
	}
	if ( rv )
	  break;
#endif
      }
    }
    if ( rv )
      break;
  }
  poptFreeContext( optCon );
exit:
#ifndef IDLE_RUN
  if ( bcu_dev_fini( &bcu.dev ) ) {
    rv = errno;
    perror( bcu.dev.path );
  }
#endif
  return rv;
}
