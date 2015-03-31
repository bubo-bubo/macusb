from ctypes import *

STRING = c_char_p
_libraries = {}
_libraries['libbcu.so'] = CDLL('libbcu.so')


BCU_LE98_TEST_ADDR = 2
BCU_LE98_CMD_ADDR = 1
BCU_LE98_STATUS_ADDR = 0
BCU_LE98_NAF_CONTROL_CMD = 7
BCU_LE98_NAF_WRITE_CMD = 6
BCU_LE98_NAF_READ_CMD = 5
BCU_LE98_MODE_SET_CMD = 2
BCU_LE98_MODE_GET_CMD = 1
BCU_LE98_INIT_CTL_CMD = 0
MACUSB_STATE_END = 9
MACUSB_STATE_ERR = 8
MACUSB_STATE_CONF = 7
MACUSB_STATE_ASYNC_URB_TOUT = 6
MACUSB_STATE_ASYNC_MAC_TOUT = 5
MACUSB_STATE_ASYNC_BUSY = 4
MACUSB_STATE_SYNC_READ = 3
MACUSB_STATE_SYNC_BUSY = 2
MACUSB_STATE_SYNC_WRITE = 1
MACUSB_STATE_FREE = 0
BCU_LE98_SDR_READ_CMD = 12
BCU_LE98_SDR_START_CMD = 11
BCU_LE98_SDR_STOP_CMD = 13
BCU_LE98_MISS_SFSDR_MODE = 448
BCU_LE98_MISS_AFSDR_MODE = 384
BCU_LE98_MISS_AFSNR_MODE = 320
BCU_LE98_MISS_RSV_MODE = 256
BCU_LE98_MISS_AUTO_MODE = 192
BCU_LE98_MISS_SDR_MODE = 128
BCU_LE98_MISS_SNR_MODE = 64
BCU_LE98_MISS_ADDR_MODE = 0
BCU_LE98_SNR_STOP_CMD = 10
BCU_LE98_SNR_READ_CMD = 9
BCU_LE98_SNR_START_CMD = 8
BCU_LE98_REG_WRITE_CMD = 4
BCU_LE98_REG_READ_CMD = 3
BCU_LE98_SYNC_MODE = 1
BCU_LE98_SYNC_ADDR = 5
BCU_LE98_DATA_ADDR = 4
BCU_LE98_MISS_MODE_ADDR = 3
class macusb_buffer(Structure):
    pass
u_int8_t = c_ubyte
u_int32_t = c_uint
int32_t = c_int32
macusb_buffer._pack_ = 1
macusb_buffer._fields_ = [
    ('tx_buf', u_int8_t * 512),
    ('tx_bytes', u_int32_t),
    ('rx_buf', u_int8_t * 512),
    ('rx_bytes', u_int32_t),
    ('rx_eot_offset', u_int32_t),
    ('rx_urbstatus', int32_t),
    ('state', u_int32_t),
]
macusb_buffer_t = macusb_buffer
class macusb_sync_cfg(Structure):
    pass
macusb_sync_cfg._pack_ = 1
macusb_sync_cfg._fields_ = [
    ('tx_tout', u_int32_t),
    ('rx_tout', u_int32_t),
]
macusb_sync_cfg_t = macusb_sync_cfg
class macusb_async_cfg(Structure):
    pass
macusb_async_cfg._pack_ = 1
macusb_async_cfg._fields_ = [
    ('urb_num', u_int32_t),
    ('mac_tout', u_int32_t),
    ('urb_tout', u_int32_t),
]
macusb_async_cfg_t = macusb_async_cfg

# values for enumeration 'macusb_dev_state_t'
macusb_dev_state_t = c_int # enum
class macusb_pipe_status(Structure):
    pass
u_int64_t = c_ulong
macusb_pipe_status._pack_ = 1
macusb_pipe_status._fields_ = [
    ('bytes', u_int64_t),
    ('mac_tout_urbs', u_int64_t),
    ('completed_urbs', u_int64_t),
    ('size_mismatch_urbs', u_int64_t),
    ('bad_status_urbs', u_int64_t),
    ('urb_occupancy', u_int64_t * 10),
    ('busy_urbs', u_int32_t),
]
macusb_pipe_status_t = macusb_pipe_status
class macusb_status(Structure):
    pass
macusb_status._pack_ = 1
macusb_status._fields_ = [
    ('state', u_int32_t),
    ('err', int32_t),
    ('bulk_out', macusb_pipe_status),
    ('bulk_in', macusb_pipe_status),
]
macusb_status_t = macusb_status
class macusb_rq_rx_prepacket(Structure):
    pass
u_int16_t = c_ushort
macusb_rq_rx_prepacket._fields_ = [
    ('dummy1', u_int16_t),
    ('len', u_int16_t, 10),
    ('dummy2', u_int16_t, 6),
    ('eot', u_int32_t),
]
macusb_rq_rx_prepacket_t = macusb_rq_rx_prepacket

# values for enumeration 'bcu_le98_reg_addr'
bcu_le98_reg_addr = c_int # enum
bcu_le98_reg_addr_t = bcu_le98_reg_addr

# values for enumeration 'bcu_le98_miss_mode'
bcu_le98_miss_mode = c_int # enum
bcu_le98_miss_mode_t = bcu_le98_miss_mode

# values for enumeration 'bcu_le98_cmd'
bcu_le98_cmd = c_int # enum
bcu_le98_cmd_t = bcu_le98_cmd

# values for enumeration 'bcu_le98_transfer_flags'
bcu_le98_transfer_flags = c_int # enum
bcu_le98_transfer_flags_t = bcu_le98_transfer_flags
class bcu_le98(Structure):
    pass
bcu_le98._fields_ = [
    ('id', c_int),
    ('dev_dir', STRING),
    ('path', c_char * 256),
    ('fd', c_int),
    ('transfer_flags', c_uint),
]
bcu_le98_t = bcu_le98
bcu_le98_init = _libraries['libbcu.so'].bcu_le98_init
bcu_le98_init.restype = None
bcu_le98_init.argtypes = [POINTER(bcu_le98_t)]
bcu_le98_fini = _libraries['libbcu.so'].bcu_le98_fini
bcu_le98_fini.restype = c_int
bcu_le98_fini.argtypes = [POINTER(bcu_le98_t)]
bcu_le98_open_id = _libraries['libbcu.so'].bcu_le98_open_id
bcu_le98_open_id.restype = c_int
bcu_le98_open_id.argtypes = [POINTER(bcu_le98_t), c_uint]
bcu_le98_open_path = _libraries['libbcu.so'].bcu_le98_open_path
bcu_le98_open_path.restype = c_int
bcu_le98_open_path.argtypes = [POINTER(bcu_le98_t), STRING]
bcu_le98_close = _libraries['libbcu.so'].bcu_le98_close
bcu_le98_close.restype = c_int
bcu_le98_close.argtypes = [POINTER(bcu_le98_t)]
bcu_le98_status = _libraries['libbcu.so'].bcu_le98_status
bcu_le98_status.restype = c_int
bcu_le98_status.argtypes = [POINTER(bcu_le98_t), POINTER(macusb_status_t)]
bcu_le98_reset = _libraries['libbcu.so'].bcu_le98_reset
bcu_le98_reset.restype = c_int
bcu_le98_reset.argtypes = [POINTER(bcu_le98_t)]
bcu_le98_reset_ctl = _libraries['libbcu.so'].bcu_le98_reset_ctl
bcu_le98_reset_ctl.restype = c_int
bcu_le98_reset_ctl.argtypes = [POINTER(bcu_le98_t)]
bcu_le98_clear = _libraries['libbcu.so'].bcu_le98_clear
bcu_le98_clear.restype = c_int
bcu_le98_clear.argtypes = [POINTER(bcu_le98_t)]
bcu_le98_get_sync_cfg = _libraries['libbcu.so'].bcu_le98_get_sync_cfg
bcu_le98_get_sync_cfg.restype = c_int
bcu_le98_get_sync_cfg.argtypes = [POINTER(bcu_le98_t), POINTER(macusb_sync_cfg_t)]
bcu_le98_set_sync_cfg = _libraries['libbcu.so'].bcu_le98_set_sync_cfg
bcu_le98_set_sync_cfg.restype = c_int
bcu_le98_set_sync_cfg.argtypes = [POINTER(bcu_le98_t), POINTER(macusb_sync_cfg_t)]
bcu_le98_get_async_cfg = _libraries['libbcu.so'].bcu_le98_get_async_cfg
bcu_le98_get_async_cfg.restype = c_int
bcu_le98_get_async_cfg.argtypes = [POINTER(bcu_le98_t), POINTER(macusb_async_cfg_t)]
bcu_le98_set_async_cfg = _libraries['libbcu.so'].bcu_le98_set_async_cfg
bcu_le98_set_async_cfg.restype = c_int
bcu_le98_set_async_cfg.argtypes = [POINTER(bcu_le98_t), POINTER(macusb_async_cfg_t)]
bcu_le98_exec_cmd = _libraries['libbcu.so'].bcu_le98_exec_cmd
bcu_le98_exec_cmd.restype = c_int
bcu_le98_exec_cmd.argtypes = [POINTER(bcu_le98_t), bcu_le98_cmd_t, POINTER(u_int16_t), POINTER(u_int16_t), POINTER(u_int16_t)]
bcu_le98_exec_buf = _libraries['libbcu.so'].bcu_le98_exec_buf
bcu_le98_exec_buf.restype = c_int
bcu_le98_exec_buf.argtypes = [POINTER(bcu_le98_t), POINTER(macusb_buffer_t)]
bcu_le98_encode_cmd = _libraries['libbcu.so'].bcu_le98_encode_cmd
bcu_le98_encode_cmd.restype = c_int
bcu_le98_encode_cmd.argtypes = [POINTER(macusb_buffer_t), bcu_le98_cmd_t, u_int16_t, u_int16_t]
bcu_le98_decode_cmd = _libraries['libbcu.so'].bcu_le98_decode_cmd
bcu_le98_decode_cmd.restype = c_int
bcu_le98_decode_cmd.argtypes = [POINTER(macusb_buffer_t), POINTER(c_uint), bcu_le98_cmd_t, POINTER(u_int16_t), POINTER(u_int16_t), POINTER(u_int16_t)]
bcu_le98_state_str = _libraries['libbcu.so'].bcu_le98_state_str
bcu_le98_state_str.restype = STRING
bcu_le98_state_str.argtypes = [c_uint]
_ATFILE_SOURCE = 1 # Variable c_int '1'
__USE_XOPEN2KXSI = 1 # Variable c_int '1'
__SIZEOF_PTHREAD_BARRIERATTR_T = 4 # Variable c_int '4'
__LITTLE_ENDIAN = 1234 # Variable c_int '1234'
__GNU_LIBRARY__ = 6 # Variable c_int '6'
_BITS_TYPESIZES_H = 1 # Variable c_int '1'
MACUSB_ASYNC_MAX_URBNUM = 1024 # Variable c_int '1024'
__USE_XOPEN = 1 # Variable c_int '1'
__USE_LARGEFILE64 = 1 # Variable c_int '1'
__time_t_defined = 1 # Variable c_int '1'
__USE_XOPEN2K8 = 1 # Variable c_int '1'
_STRUCT_TIMEVAL = 1 # Variable c_int '1'
__USE_POSIX2 = 1 # Variable c_int '1'
__SIZEOF_PTHREAD_RWLOCKATTR_T = 8 # Variable c_int '8'
__SIZEOF_PTHREAD_CONDATTR_T = 4 # Variable c_int '4'
MACUSB_VERSION_CODE = 65538 # Variable c_int '65538'
__STDC_IEC_559__ = 1 # Variable c_int '1'
_BITS_PTHREADTYPES_H = 1 # Variable c_int '1'
MACUSB_ASYNC_URB_TOUT = 3000 # Variable c_int '3000'
__USE_ATFILE = 1 # Variable c_int '1'
__GLIBC_HAVE_LONG_LONG = 1 # Variable c_int '1'
__NFDBITS = 64 # Variable c_int '64'
_SYS_SELECT_H = 1 # Variable c_int '1'
MACUSB_EOT = 4294967295L # Variable c_uint '4294967295u'
__SIZEOF_PTHREAD_MUTEXATTR_T = 4 # Variable c_int '4'
_POSIX_SOURCE = 1 # Variable c_int '1'
_ISOC95_SOURCE = 1 # Variable c_int '1'
_ISOC99_SOURCE = 1 # Variable c_int '1'
MACUSB_ASYNC_URBNUM = 120 # Variable c_int '120'
__USE_POSIX = 1 # Variable c_int '1'
MACUSB_ASYNC_MAX_MAC_TOUT = 250 # Variable c_int '250'
__clock_t_defined = 1 # Variable c_int '1'
__USE_POSIX199309 = 1 # Variable c_int '1'
__GLIBC_MINOR__ = 13 # Variable c_int '13'
__clockid_t_defined = 1 # Variable c_int '1'
__timer_t_defined = 1 # Variable c_int '1'
BCU_LE98_TANSFER_FLAGS = 0 # Variable c_int '0'
_BCU_LE98_H_ = 1 # Variable c_int '1'
__BIT_TYPES_DEFINED__ = 1 # Variable c_int '1'
_SVID_SOURCE = 1 # Variable c_int '1'
__USE_XOPEN2K = 1 # Variable c_int '1'
__SIZEOF_PTHREAD_BARRIER_T = 32 # Variable c_int '32'
__FD_SETSIZE = 1024 # Variable c_int '1024'
_SYS_TYPES_H = 1 # Variable c_int '1'
MACUSB_SYNC_RXTOUT = 3000 # Variable c_int '3000'
__timespec_defined = 1 # Variable c_int '1'
__USE_GNU = 1 # Variable c_int '1'
__USE_BSD = 1 # Variable c_int '1'
_SIGSET_NWORDS = 16L # Variable c_ulong '16ul'
_LARGEFILE_SOURCE = 1 # Variable c_int '1'
_POSIX_C_SOURCE = 200809 # Variable c_long '200809l'
_SIGSET_H_types = 1 # Variable c_int '1'
__USE_SVID = 1 # Variable c_int '1'
__SIZEOF_PTHREAD_MUTEX_T = 40 # Variable c_int '40'
__USE_UNIX98 = 1 # Variable c_int '1'
__USE_ANSI = 1 # Variable c_int '1'
__USE_MISC = 1 # Variable c_int '1'
MACUSB_MINOR_BASE = 192 # Variable c_int '192'
MACUSB_VERSION_STR = '1.0.2' # Variable STRING '(const char*)"1.0.2"'
_ENDIAN_H = 1 # Variable c_int '1'
__PTHREAD_MUTEX_HAVE_PREV = 1 # Variable c_int '1'
__USE_FORTIFY_LEVEL = 2 # Variable c_int '2'
__SIZEOF_PTHREAD_RWLOCK_T = 56 # Variable c_int '56'
BCU_LE98_PATH_BUFSIZE = 256 # Variable c_int '256'
_BITS_BYTESWAP_H = 1 # Variable c_int '1'
__STDC_ISO_10646__ = 200009 # Variable c_long '200009l'
__STDC_IEC_559_COMPLEX__ = 1 # Variable c_int '1'
_SYS_SYSMACROS_H = 1 # Variable c_int '1'
__USE_XOPEN_EXTENDED = 1 # Variable c_int '1'
_MACUSB_H_ = 1 # Variable c_int '1'
MACUSB_IOC_MAGIC = 'x' # Variable c_char "'x'"
__USE_LARGEFILE = 1 # Variable c_int '1'
__USE_EXTERN_INLINES = 1 # Variable c_int '1'
__SIZEOF_PTHREAD_COND_T = 48 # Variable c_int '48'
_FEATURES_H = 1 # Variable c_int '1'
__USE_POSIX199506 = 1 # Variable c_int '1'
_BITS_TYPES_H = 1 # Variable c_int '1'
__BIG_ENDIAN = 4321 # Variable c_int '4321'
__WORDSIZE_COMPAT32 = 1 # Variable c_int '1'
_XOPEN_SOURCE_EXTENDED = 1 # Variable c_int '1'
MACUSB_SYNC_TXTOUT = 3000 # Variable c_int '3000'
__USE_XOPEN2K8XSI = 1 # Variable c_int '1'
__WORDSIZE = 64 # Variable c_int '64'
__PDP_ENDIAN = 3412 # Variable c_int '3412'
MACUSB_NODE_PREFIX = 'macusb' # Variable STRING '(const char*)"macusb"'
_SYS_CDEFS_H = 1 # Variable c_int '1'
_LARGEFILE64_SOURCE = 1 # Variable c_int '1'
_XOPEN_SOURCE = 700 # Variable c_int '700'
__SIZEOF_PTHREAD_ATTR_T = 56 # Variable c_int '56'
MACUSB_PRODUCT_ID = 4098 # Variable c_int '4098'
MACUSB_URB_OCCUPANCY_HISTO_SIZE = 10 # Variable c_int '10'
__USE_ISOC95 = 1 # Variable c_int '1'
MACUSB_TXBUFSIZE = 512 # Variable c_int '512'
__GLIBC__ = 2 # Variable c_int '2'
MACUSB_IOC_MAXNR = 15 # Variable c_int '15'
__USE_ISOC99 = 1 # Variable c_int '1'
MACUSB_RXBUFSIZE = 512 # Variable c_int '512'
_BSD_SOURCE = 1 # Variable c_int '1'
__FD_ZERO_STOS = 'stosq' # Variable STRING '(const char*)"stosq"'
MACUSB_VENDOR_ID = 1204 # Variable c_int '1204'
pthread_t = c_ulong
class __pthread_internal_list(Structure):
    pass
__pthread_internal_list._fields_ = [
    ('__prev', POINTER(__pthread_internal_list)),
    ('__next', POINTER(__pthread_internal_list)),
]
__pthread_list_t = __pthread_internal_list
class __pthread_mutex_s(Structure):
    pass
__pthread_mutex_s._fields_ = [
    ('__lock', c_int),
    ('__count', c_uint),
    ('__owner', c_int),
    ('__nusers', c_uint),
    ('__kind', c_int),
    ('__spins', c_int),
    ('__list', __pthread_list_t),
]
class N14pthread_cond_t3DOT_7E(Structure):
    pass
N14pthread_cond_t3DOT_7E._fields_ = [
    ('__lock', c_int),
    ('__futex', c_uint),
    ('__total_seq', c_ulonglong),
    ('__wakeup_seq', c_ulonglong),
    ('__woken_seq', c_ulonglong),
    ('__mutex', c_void_p),
    ('__nwaiters', c_uint),
    ('__broadcast_seq', c_uint),
]
pthread_key_t = c_uint
pthread_once_t = c_int
class N16pthread_rwlock_t4DOT_10E(Structure):
    pass
N16pthread_rwlock_t4DOT_10E._fields_ = [
    ('__lock', c_int),
    ('__nr_readers', c_uint),
    ('__readers_wakeup', c_uint),
    ('__writer_wakeup', c_uint),
    ('__nr_readers_queued', c_uint),
    ('__nr_writers_queued', c_uint),
    ('__writer', c_int),
    ('__shared', c_int),
    ('__pad1', c_ulong),
    ('__pad2', c_ulong),
    ('__flags', c_uint),
]
pthread_spinlock_t = c_int
__sig_atomic_t = c_int
class __sigset_t(Structure):
    pass
__sigset_t._fields_ = [
    ('__val', c_ulong * 16),
]
class timeval(Structure):
    pass
__time_t = c_long
__suseconds_t = c_long
timeval._fields_ = [
    ('tv_sec', __time_t),
    ('tv_usec', __suseconds_t),
]
__u_char = c_ubyte
__u_short = c_ushort
__u_int = c_uint
__u_long = c_ulong
__int8_t = c_byte
__uint8_t = c_ubyte
__int16_t = c_short
__uint16_t = c_ushort
__int32_t = c_int
__uint32_t = c_uint
__int64_t = c_long
__uint64_t = c_ulong
__quad_t = c_long
__u_quad_t = c_ulong
__dev_t = c_ulong
__uid_t = c_uint
__gid_t = c_uint
__ino_t = c_ulong
__ino64_t = c_ulong
__mode_t = c_uint
__nlink_t = c_ulong
__off_t = c_long
__off64_t = c_long
__pid_t = c_int
class __fsid_t(Structure):
    pass
__fsid_t._fields_ = [
    ('__val', c_int * 2),
]
__clock_t = c_long
__rlim_t = c_ulong
__rlim64_t = c_ulong
__id_t = c_uint
__useconds_t = c_uint
__daddr_t = c_int
__swblk_t = c_long
__key_t = c_int
__clockid_t = c_int
__timer_t = c_void_p
__blksize_t = c_long
__blkcnt_t = c_long
__blkcnt64_t = c_long
__fsblkcnt_t = c_ulong
__fsblkcnt64_t = c_ulong
__fsfilcnt_t = c_ulong
__fsfilcnt64_t = c_ulong
__ssize_t = c_long
__loff_t = __off64_t
__qaddr_t = POINTER(__quad_t)
__caddr_t = STRING
__intptr_t = c_long
__socklen_t = c_uint
sigset_t = __sigset_t
__fd_mask = c_long
class fd_set(Structure):
    pass
fd_set._fields_ = [
    ('fds_bits', __fd_mask * 16),
]
fd_mask = __fd_mask
select = _libraries['libbcu.so'].select
select.restype = c_int
select.argtypes = [c_int, POINTER(fd_set), POINTER(fd_set), POINTER(fd_set), POINTER(timeval)]
class timespec(Structure):
    pass
timespec._fields_ = [
    ('tv_sec', __time_t),
    ('tv_nsec', c_long),
]
pselect = _libraries['libbcu.so'].pselect
pselect.restype = c_int
pselect.argtypes = [c_int, POINTER(fd_set), POINTER(fd_set), POINTER(fd_set), POINTER(timespec), POINTER(__sigset_t)]
gnu_dev_major = _libraries['libbcu.so'].gnu_dev_major
gnu_dev_major.restype = c_uint
gnu_dev_major.argtypes = [c_ulonglong]
gnu_dev_minor = _libraries['libbcu.so'].gnu_dev_minor
gnu_dev_minor.restype = c_uint
gnu_dev_minor.argtypes = [c_ulonglong]
gnu_dev_makedev = _libraries['libbcu.so'].gnu_dev_makedev
gnu_dev_makedev.restype = c_ulonglong
gnu_dev_makedev.argtypes = [c_uint, c_uint]
u_char = __u_char
u_short = __u_short
u_int = __u_int
u_long = __u_long
quad_t = __quad_t
u_quad_t = __u_quad_t
fsid_t = __fsid_t
loff_t = __loff_t
ino_t = __ino_t
ino64_t = __ino64_t
dev_t = __dev_t
gid_t = __gid_t
mode_t = __mode_t
nlink_t = __nlink_t
uid_t = __uid_t
off_t = __off_t
off64_t = __off64_t
pid_t = __pid_t
id_t = __id_t
ssize_t = __ssize_t
daddr_t = __daddr_t
caddr_t = __caddr_t
key_t = __key_t
useconds_t = __useconds_t
suseconds_t = __suseconds_t
ulong = c_ulong
ushort = c_ushort
uint = c_uint
int8_t = c_int8
int16_t = c_int16
int64_t = c_int64
register_t = c_long
blksize_t = __blksize_t
blkcnt_t = __blkcnt_t
fsblkcnt_t = __fsblkcnt_t
fsfilcnt_t = __fsfilcnt_t
blkcnt64_t = __blkcnt64_t
fsblkcnt64_t = __fsblkcnt64_t
fsfilcnt64_t = __fsfilcnt64_t
clock_t = __clock_t
time_t = __time_t
clockid_t = __clockid_t
timer_t = __timer_t
size_t = c_ulong
__all__ = ['__uint16_t', '__pthread_mutex_s', '_ATFILE_SOURCE',
           'BCU_LE98_SDR_STOP_CMD', '_POSIX_C_SOURCE',
           '__LITTLE_ENDIAN', '__WORDSIZE',
           'MACUSB_STATE_ASYNC_URB_TOUT', 'bcu_le98_reg_addr',
           'u_short', 'BCU_LE98_MISS_SDR_MODE', '__GNU_LIBRARY__',
           '_BITS_TYPESIZES_H', '__fsid_t', 'MACUSB_ASYNC_MAX_URBNUM',
           '__off64_t', 'size_t', '__USE_XOPEN', '__USE_LARGEFILE64',
           'BCU_LE98_MISS_AUTO_MODE', '__USE_XOPEN2KXSI', '__key_t',
           'bcu_le98_open_path', '__USE_XOPEN2K8', '_STRUCT_TIMEVAL',
           '__USE_POSIX2', 'macusb_status_t', '__time_t', 'uid_t',
           '__STDC_IEC_559_COMPLEX__', '__timer_t_defined',
           '__STDC_ISO_10646__', 'MACUSB_STATE_END',
           'BCU_LE98_REG_READ_CMD', '__ino64_t',
           '__SIZEOF_PTHREAD_CONDATTR_T', '__qaddr_t', '__NFDBITS',
           'gnu_dev_minor', '__mode_t', 'gnu_dev_major', '__loff_t',
           'off64_t', 'blksize_t', '_SYS_SELECT_H', '__int16_t',
           'bcu_le98_get_async_cfg', '__STDC_IEC_559__',
           'macusb_sync_cfg', '_BITS_PTHREADTYPES_H', 'u_char',
           'MACUSB_ASYNC_URB_TOUT', '__USE_ATFILE', 'u_int64_t',
           'u_int16_t', '__u_quad_t', 'macusb_async_cfg_t', '__pid_t',
           'MACUSB_VERSION_CODE', 'bcu_le98_transfer_flags',
           'bcu_le98_reset', 'bcu_le98_encode_cmd',
           'BCU_LE98_REG_WRITE_CMD', 'sigset_t', 'MACUSB_EOT',
           '__SIZEOF_PTHREAD_MUTEXATTR_T', '_POSIX_SOURCE',
           '_ISOC95_SOURCE', 'suseconds_t', '__int32_t',
           'bcu_le98_status', 'fd_mask', 'MACUSB_ASYNC_URBNUM',
           '__USE_POSIX', 'MACUSB_STATE_CONF', 'pthread_once_t',
           'gnu_dev_makedev', 'BCU_LE98_STATUS_ADDR',
           'MACUSB_ASYNC_MAX_MAC_TOUT', 'BCU_LE98_INIT_CTL_CMD',
           'bcu_le98_transfer_flags_t', 'mode_t',
           'bcu_le98_set_async_cfg', 'fsid_t', '__nlink_t', '__off_t',
           'bcu_le98_open_id', 'bcu_le98_fini', 'MACUSB_NODE_PREFIX',
           '__clock_t_defined', '_LARGEFILE64_SOURCE',
           'BCU_LE98_MISS_SNR_MODE', 'clock_t',
           'BCU_LE98_SNR_STOP_CMD', 'macusb_async_cfg',
           'macusb_rq_rx_prepacket', '__USE_POSIX199309', '__id_t',
           '__sigset_t', '__clockid_t', '__useconds_t', 'ulong',
           '__GLIBC_MINOR__', 'MACUSB_STATE_SYNC_WRITE',
           'bcu_le98_state_str', '__timer_t', '__clockid_t_defined',
           '__fsfilcnt64_t', '_ISOC99_SOURCE', 'id_t',
           'bcu_le98_get_sync_cfg', 'BCU_LE98_TANSFER_FLAGS',
           '_BCU_LE98_H_', '__pthread_internal_list',
           'macusb_pipe_status_t', 'BCU_LE98_SYNC_ADDR', 'caddr_t',
           'u_int32_t', '_SVID_SOURCE', '__USE_XOPEN2K',
           '__SIZEOF_PTHREAD_BARRIER_T', '__uint32_t', '__FD_SETSIZE',
           '_SYS_TYPES_H', 'N14pthread_cond_t3DOT_7E',
           '__time_t_defined', 'MACUSB_STATE_ERR',
           'MACUSB_STATE_FREE', '__intptr_t', 'ushort',
           'macusb_dev_state_t', 'bcu_le98_init',
           'MACUSB_SYNC_RXTOUT', '__timespec_defined', '__uint64_t',
           '__USE_GNU', 'BCU_LE98_MISS_MODE_ADDR', 'key_t',
           'BCU_LE98_NAF_READ_CMD', '__USE_ISOC95', '__blkcnt_t',
           'pthread_t', 'clockid_t', '__GLIBC_HAVE_LONG_LONG',
           'BCU_LE98_NAF_CONTROL_CMD', 'fd_set',
           'BCU_LE98_SNR_READ_CMD', '__ino_t', 'bcu_le98_cmd',
           '__rlim64_t', 'ino_t', 'BCU_LE98_SDR_START_CMD',
           'bcu_le98_set_sync_cfg', '_LARGEFILE_SOURCE',
           '__SIZEOF_PTHREAD_BARRIERATTR_T', '_SIGSET_H_types',
           '__fd_mask', 'uint', 'int32_t', 'MACUSB_STATE_SYNC_BUSY',
           'N16pthread_rwlock_t4DOT_10E', '__blksize_t',
           'BCU_LE98_SNR_START_CMD', '__USE_SVID',
           'MACUSB_STATE_ASYNC_BUSY', '__SIZEOF_PTHREAD_MUTEX_T',
           '__USE_UNIX98', 'macusb_buffer_t', '__USE_ANSI',
           'blkcnt_t', '__USE_MISC', '__GLIBC__', 'select',
           '__BIT_TYPES_DEFINED__', 'bcu_le98_exec_cmd', 'bcu_le98',
           'u_quad_t', '__ssize_t', 'register_t', '__u_long',
           'fsfilcnt64_t', 'MACUSB_STATE_ASYNC_MAC_TOUT',
           'MACUSB_VERSION_STR', '__daddr_t', 'ino64_t',
           '_BITS_TYPES_H', '__sig_atomic_t', 'BCU_LE98_TEST_ADDR',
           '_ENDIAN_H', 'fsblkcnt64_t', '__uint8_t',
           'MACUSB_STATE_SYNC_READ', 'BCU_LE98_SYNC_MODE',
           '__SIZEOF_PTHREAD_RWLOCK_T', 'BCU_LE98_PATH_BUFSIZE',
           '_BITS_BYTESWAP_H', '__socklen_t', 'BCU_LE98_MODE_GET_CMD',
           '__caddr_t', '__blkcnt64_t', '__dev_t',
           'BCU_LE98_SDR_READ_CMD', 'fsblkcnt_t', 'off_t', 'gid_t',
           'BCU_LE98_DATA_ADDR', '_SYS_SYSMACROS_H',
           '__USE_XOPEN_EXTENDED', '__suseconds_t', 'pid_t',
           '_MACUSB_H_', 'timer_t', 'quad_t', 'u_long',
           '__USE_LARGEFILE', 'macusb_rq_rx_prepacket_t',
           '__SIZEOF_PTHREAD_COND_T', '_FEATURES_H', 'macusb_status',
           'pthread_key_t', 'int16_t', 'BCU_LE98_NAF_WRITE_CMD',
           'blkcnt64_t', 'u_int8_t', 'loff_t',
           'BCU_LE98_MISS_SFSDR_MODE', '__USE_POSIX199506',
           '__USE_BSD', '__BIG_ENDIAN', 'pthread_spinlock_t',
           '__u_char', 'bcu_le98_cmd_t', '__WORDSIZE_COMPAT32',
           'pselect', '__u_int', 'int64_t', '__fsblkcnt_t',
           '__rlim_t', '__fsfilcnt_t', 'bcu_le98_reset_ctl', 'time_t',
           '__pthread_list_t', 'bcu_le98_miss_mode', 'bcu_le98_t',
           'bcu_le98_exec_buf', 'bcu_le98_reg_addr_t',
           '_XOPEN_SOURCE_EXTENDED', '__quad_t', 'MACUSB_SYNC_TXTOUT',
           'timeval', 'daddr_t', '__PTHREAD_MUTEX_HAVE_PREV',
           '__USE_XOPEN2K8XSI', '__SIZEOF_PTHREAD_RWLOCKATTR_T',
           '__PDP_ENDIAN', '__u_short', '__USE_FORTIFY_LEVEL',
           '__fsblkcnt64_t', 'bcu_le98_close', '_SYS_CDEFS_H',
           'fsfilcnt_t', '__gid_t', 'MACUSB_MINOR_BASE',
           'MACUSB_IOC_MAGIC', '_XOPEN_SOURCE', 'macusb_pipe_status',
           '__SIZEOF_PTHREAD_ATTR_T', 'MACUSB_PRODUCT_ID',
           'MACUSB_URB_OCCUPANCY_HISTO_SIZE', 'bcu_le98_miss_mode_t',
           'bcu_le98_decode_cmd', 'timespec', '_SIGSET_NWORDS',
           'BCU_LE98_MISS_AFSNR_MODE', 'MACUSB_TXBUFSIZE', 'u_int',
           'useconds_t', 'nlink_t', 'BCU_LE98_MODE_SET_CMD',
           'MACUSB_IOC_MAXNR', '__swblk_t', '__USE_ISOC99',
           'BCU_LE98_CMD_ADDR', '__USE_EXTERN_INLINES', 'int8_t',
           'macusb_buffer', 'BCU_LE98_MISS_ADDR_MODE',
           'BCU_LE98_MISS_RSV_MODE', 'BCU_LE98_MISS_AFSDR_MODE',
           'macusb_sync_cfg_t', 'bcu_le98_clear', 'MACUSB_RXBUFSIZE',
           '__int64_t', '_BSD_SOURCE', 'ssize_t', '__clock_t',
           'dev_t', '__uid_t', '__int8_t', '__FD_ZERO_STOS',
           'MACUSB_VENDOR_ID']
