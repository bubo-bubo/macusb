MAC USB driver and library

Driver
======

The driver implements request layer of Bus controller - USB (BCU) protocol
stack.

Request layer:

  - Implements data exchange with USB pipes.

  - Provides 15 usec average latency of request execution with
    Mechanism of Accelerated Completion (MAC).
    URB completion handler is called on microframe edges (microframe
    duration is 125 usec) so a lower limit of average execution time is
    about 63 usec. MAC is a bypass of URB completion handler.

Request is bound one-for-one with a set of two URBs:

  1. TX URB (request transmission, bulk-out URB)
  2. RX URB (answer receiving, bulk-in URB)

Request transmission is implemented with one TX URB with a data block in TX
buffer encoded completely by upper layer in BCU protocol stack.
No limits on format of data block are imposed by request layer.

Answer receiving is implemented with one RX URB with a data block in RX
buffer ended with 4-byte trailer decoded by request layer. The trailer is
used by MAC. Received data block is to be decoded by upper layer in BCU
protocol stack.

The driver implements access to a MAC USB device with character
special file from devfs (device node). Each device attached to USB
and enumerated is assigned with a unique device node name. The name matches
pattern /dev/macusbN-X, where N and X are natural numbers.

At any moment device node state is set to one of five states:

  MACUSB_STATE_FREE            free, no IO in progress
  MACUSB_STATE_SYNC_WRITE      sync TX in progress
  MACUSB_STATE_SYNC_BUSY       sync TX done successfully
  MACUSB_STATE_SYNC_READ       sync RX in progress
  MACUSB_STATE_ASYNC_BUSY      executing request, completion flag loop is
                               running
  MACUSB_STATE_ASYNC_MAC_TOUT  executing request, completion flag loop has
                               finished, waiting for RX URB completion
                               handler
  MACUSB_STATE_ASYNC_URB_TOUT  RX URB completion handler timeout.
                               The state is changed to:
                                 * FREE by successful RX URB completion
                                   handler,
                                 * ERR by failed RX URB completion handler
                                   or by ioctl(MACUSB_IOC_RQ_STOP)
  MACUSB_STATE_CONF            configuration is in proggress
  MACUSB_STATE_ERR             failed to execute request
  MACUSB_STATE_END             end of device lifecycle

State change is synchronized with spin lock to allow some operations
be run concurrently on SMP systems for the same device node.

Example of asynchronous request:

  const char *path = "/dev/macusb/macusb1-0";
  fd = open( path )
  if ( ioctl( fd, MACUSB_IOC_RQ_EOT, &cmd_buffer ) )
    perror( path );
  close( fd );

Example of synchronous request:

  const char *path = "/dev/macusb/macusb1-0";
  fd = open( path )
  // send commands
  if ( (bytes=write( fd, &out_buffer, count )) < 0 )
    perror( path );
  // receive answer
  if ( (bytes=read( fd, &in_buffer, in_buffer_size )) < 0 )
    perror( path );
  close( fd );

NB: Atomicity of request execution is guaranteed, order of RX URBs is
    congruent to order of TX URBs for concurrent asynchronous AND
    synchronous requests always. The driver is thread-safe as of
    concurrent syscalls.

NB2: The driver is NOT thread-safe as of order of concurrent requests.
    Threads executing requests on the same device concurrently must be
    synchronized in user space.

NB3: If device state was changed to MACUSB_STATE_ERR the device should be
    at least cleared with MACUSB_IOC_CLEAR to erase possibly suspended
    bulk-out URB. Clearing is safe for all finished requests because
    completion handler doesn't change RX data of MAC'ed RX URBs.
    It is up to user to reset device or not somehow.

NB4:  ioctl arg is of type u_int64_t to make cmd name invariant by
      32/64-bit process context

NB5: pointers in interface structures should have 64-bit width by the same
     reason


Library
=======

The library implements register and command layers of BCU protocol stack.

The register layer implements elemental operations with bus controller registers:

  - errors reported by controller,
  - TX/RX packets for operations with registers,
  - types of operations with registers,
  - macros to en-/de-code register operations,

The command layer implements command layer of BCU protocol stack for MISS:

  - addresses of LE98 registers,
  - MISS work modes,
  - LE98 commands
  - funcs to encode, execute, decode LE98 commands, wrap macusb driver
    ioctl's

One or some MISS commands packed in a buffer are executed as a request to
controller. Request can contain one or more register operations and is
executed as a single whole. No intermediate state can be saved and
no execution can be continued later.


# macusb
