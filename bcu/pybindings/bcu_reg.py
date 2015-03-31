from ctypes import *

STRING = c_char_p
_libraries = {}
_libraries['libbcu.so'] = CDLL('libbcu.so')


BCU_DECODE_NO_EOT_ERR = 524288
BCU_DECODE_NO_SUCH_COMMAND_ERR = 262144
BCU_DECODE_FORMAT_ERR = 131072
BCU_DECODE_DATA_LEN_ERR = 65536
BCU_ENCODE_NO_SUCH_COMMAND_ERR = 1024
BCU_ENCODE_BUF_NOT_EMPTY_ERR = 512
BCU_ENCODE_BUF_OVERRUN_ERR = 256
BCU_CTL_BUS_TOUT_ERR = 128
BCU_CTL_LEN_ERR = 32
BCU_CTL_INIT_OK = 16
BCU_CTL_OK = 0
BCU_CTL_REG_ADDR_ERR = 64

# values for enumeration 'bcu_error'
bcu_error = c_int # enum
bcu_error_t = bcu_error
class bcu_tx_header(Structure):
    pass
u_int16_t = c_ushort
bcu_tx_header._fields_ = [
    ('reg_addr', u_int16_t, 5),
    ('reg_op1', u_int16_t, 1),
    ('reg_op2', u_int16_t, 1),
    ('reg_op3', u_int16_t, 1),
    ('init_ctl', u_int16_t, 8),
]
bcu_tx_header_t = bcu_tx_header
class bcu_tx_packet(Structure):
    pass
bcu_tx_packet._pack_ = 1
bcu_tx_packet._fields_ = [
    ('hdr', bcu_tx_header),
    ('pld', u_int16_t),
]
bcu_tx_packet_t = bcu_tx_packet
class bcu_rx_header(Structure):
    pass
bcu_rx_header._fields_ = [
    ('reg_addr', u_int16_t, 5),
    ('reg_op1', u_int16_t, 1),
    ('reg_op2', u_int16_t, 1),
    ('reg_op3', u_int16_t, 1),
    ('errors', u_int16_t, 8),
]
bcu_rx_header_t = bcu_rx_header
class bcu_rx_packet(Structure):
    pass
bcu_rx_packet._pack_ = 1
bcu_rx_packet._fields_ = [
    ('hdr', bcu_rx_header),
    ('pld', u_int16_t),
]
bcu_rx_packet_t = bcu_rx_packet
class bcu_rx_prepacket(Structure):
    pass
u_int32_t = c_uint
bcu_rx_prepacket._fields_ = [
    ('ret_code', u_int16_t),
    ('len', u_int16_t, 10),
    ('dummy', u_int16_t, 6),
    ('eot', u_int32_t),
]
bcu_rx_prepacket_t = bcu_rx_prepacket
_POSIX_C_SOURCE = 200809 # Variable c_long '200809l'
_SIGSET_H_types = 1 # Variable c_int '1'
__USE_POSIX199506 = 1 # Variable c_int '1'
__USE_XOPEN2KXSI = 1 # Variable c_int '1'
__GLIBC__ = 2 # Variable c_int '2'
_BITS_TYPES_H = 1 # Variable c_int '1'
__BIG_ENDIAN = 4321 # Variable c_int '4321'
__LITTLE_ENDIAN = 1234 # Variable c_int '1234'
__SIZEOF_PTHREAD_RWLOCK_T = 56 # Variable c_int '56'
__WORDSIZE_COMPAT32 = 1 # Variable c_int '1'
__timer_t_defined = 1 # Variable c_int '1'
_ENDIAN_H = 1 # Variable c_int '1'
__GNU_LIBRARY__ = 6 # Variable c_int '6'
_BITS_TYPESIZES_H = 1 # Variable c_int '1'
__SIZEOF_PTHREAD_MUTEX_T = 40 # Variable c_int '40'
__USE_UNIX98 = 1 # Variable c_int '1'
BCU_REG_WRITE = 1 # Variable c_int '1'
__USE_ANSI = 1 # Variable c_int '1'
__USE_LARGEFILE64 = 1 # Variable c_int '1'
_ISOC95_SOURCE = 1 # Variable c_int '1'
__USE_MISC = 1 # Variable c_int '1'
__USE_POSIX199309 = 1 # Variable c_int '1'
__BIT_TYPES_DEFINED__ = 1 # Variable c_int '1'
__SIZEOF_PTHREAD_RWLOCKATTR_T = 8 # Variable c_int '8'
__USE_XOPEN2K8 = 1 # Variable c_int '1'
__GLIBC_MINOR__ = 13 # Variable c_int '13'
__USE_POSIX2 = 1 # Variable c_int '1'
_XOPEN_SOURCE_EXTENDED = 1 # Variable c_int '1'
__GLIBC_HAVE_LONG_LONG = 1 # Variable c_int '1'
__clockid_t_defined = 1 # Variable c_int '1'
_BCU_REG_H_ = 1 # Variable c_int '1'
__USE_XOPEN2K8XSI = 1 # Variable c_int '1'
__WORDSIZE = 64 # Variable c_int '64'
__SIZEOF_PTHREAD_ATTR_T = 56 # Variable c_int '56'
_SYS_SYSMACROS_H = 1 # Variable c_int '1'
__USE_FORTIFY_LEVEL = 2 # Variable c_int '2'
__SIZEOF_PTHREAD_CONDATTR_T = 4 # Variable c_int '4'
__SIZEOF_PTHREAD_BARRIERATTR_T = 4 # Variable c_int '4'
__PDP_ENDIAN = 3412 # Variable c_int '3412'
_SYS_CDEFS_H = 1 # Variable c_int '1'
_ATFILE_SOURCE = 1 # Variable c_int '1'
BCU_REG_READ = 0 # Variable c_int '0'
BCU_ENCODE_ERROR_MASK = 65280 # Variable c_int '65280'
_LARGEFILE64_SOURCE = 1 # Variable c_int '1'
__USE_XOPEN = 1 # Variable c_int '1'
_XOPEN_SOURCE = 700 # Variable c_int '700'
BCU_INIT_CTL = 255 # Variable c_int '255'
__PTHREAD_MUTEX_HAVE_PREV = 1 # Variable c_int '1'
_SVID_SOURCE = 1 # Variable c_int '1'
__USE_XOPEN2K = 1 # Variable c_int '1'
_STRUCT_TIMEVAL = 1 # Variable c_int '1'
__STDC_IEC_559__ = 1 # Variable c_int '1'
__FD_SETSIZE = 1024 # Variable c_int '1024'
_POSIX_SOURCE = 1 # Variable c_int '1'
_BITS_PTHREADTYPES_H = 1 # Variable c_int '1'
_SYS_TYPES_H = 1 # Variable c_int '1'
__USE_ISOC95 = 1 # Variable c_int '1'
__SIZEOF_PTHREAD_BARRIER_T = 32 # Variable c_int '32'
__STDC_ISO_10646__ = 200009 # Variable c_long '200009l'
__USE_ATFILE = 1 # Variable c_int '1'
_BITS_BYTESWAP_H = 1 # Variable c_int '1'
__time_t_defined = 1 # Variable c_int '1'
BCU_CTL_ERROR_MASK = 255 # Variable c_int '255'
__STDC_IEC_559_COMPLEX__ = 1 # Variable c_int '1'
__USE_ISOC99 = 1 # Variable c_int '1'
__NFDBITS = 64 # Variable c_int '64'
_SYS_SELECT_H = 1 # Variable c_int '1'
__USE_XOPEN_EXTENDED = 1 # Variable c_int '1'
__timespec_defined = 1 # Variable c_int '1'
BCU_DECODE_ERROR_MASK = 16711680 # Variable c_int '16711680'
__USE_GNU = 1 # Variable c_int '1'
__SIZEOF_PTHREAD_MUTEXATTR_T = 4 # Variable c_int '4'
__USE_BSD = 1 # Variable c_int '1'
_SIGSET_NWORDS = 16L # Variable c_ulong '16ul'
__USE_LARGEFILE = 1 # Variable c_int '1'
__USE_EXTERN_INLINES = 1 # Variable c_int '1'
__SIZEOF_PTHREAD_COND_T = 48 # Variable c_int '48'
_ISOC99_SOURCE = 1 # Variable c_int '1'
_FEATURES_H = 1 # Variable c_int '1'
_BSD_SOURCE = 1 # Variable c_int '1'
__clock_t_defined = 1 # Variable c_int '1'
__USE_POSIX = 1 # Variable c_int '1'
__USE_SVID = 1 # Variable c_int '1'
__FD_ZERO_STOS = 'stosq' # Variable STRING '(const char*)"stosq"'
_LARGEFILE_SOURCE = 1 # Variable c_int '1'
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
int32_t = c_int32
int64_t = c_int64
u_int8_t = c_ubyte
u_int64_t = c_ulong
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
           '__USE_XOPEN2KXSI', 'bcu_rx_prepacket_t',
           '__SIZEOF_PTHREAD_BARRIERATTR_T', '__LITTLE_ENDIAN',
           '__GNU_LIBRARY__', '_BITS_TYPESIZES_H', '__fsid_t',
           'mode_t', '__off64_t', 'size_t', '__USE_XOPEN',
           '__USE_LARGEFILE64', '__GLIBC_HAVE_LONG_LONG',
           '__uint32_t', 'BCU_DECODE_NO_SUCH_COMMAND_ERR',
           '_STRUCT_TIMEVAL', '__USE_POSIX2', 'fd_set', '_BCU_REG_H_',
           '__WORDSIZE', '__ino64_t', '__SIZEOF_PTHREAD_CONDATTR_T',
           '__qaddr_t', '__NFDBITS', 'gnu_dev_minor', 'int32_t',
           'gnu_dev_major', '__loff_t', 'id_t', 'blksize_t',
           '_SYS_SELECT_H', 'daddr_t', 'bcu_rx_header',
           '_BITS_PTHREADTYPES_H', 'u_char', 'uid_t', '__USE_ATFILE',
           'u_int64_t', 'u_int16_t', '__u_quad_t', '__time_t',
           'BCU_CTL_REG_ADDR_ERR', 'BCU_INIT_CTL',
           'BCU_DECODE_FORMAT_ERR', 'sigset_t',
           '__SIZEOF_PTHREAD_MUTEXATTR_T', '_POSIX_SOURCE',
           '_ISOC95_SOURCE', '__int32_t', '_ISOC99_SOURCE', 'fd_mask',
           '__USE_POSIX', 'fsfilcnt_t', 'pthread_once_t',
           'gnu_dev_makedev', '__int16_t', '__swblk_t', '__uint64_t',
           '__ssize_t', 'gid_t', '__nlink_t', '__off_t',
           'BCU_CTL_INIT_OK', '__fd_mask', 'bcu_tx_header', '__pid_t',
           'int16_t', 'BCU_CTL_BUS_TOUT_ERR', 'clock_t',
           '__USE_EXTERN_INLINES', '__USE_POSIX199309', '__id_t',
           '__sigset_t', '__clockid_t', '__useconds_t', 'ulong',
           '__GLIBC_MINOR__', 'bcu_rx_packet', '__timer_t',
           '__clockid_t_defined', '__fsfilcnt64_t',
           '__timer_t_defined', 'off_t', 'BCU_DECODE_DATA_LEN_ERR',
           'bcu_rx_packet_t', '__pthread_internal_list', '__gid_t',
           'BCU_ENCODE_ERROR_MASK', 'u_int32_t', '_SVID_SOURCE',
           '__USE_XOPEN2K', '__SIZEOF_PTHREAD_BARRIER_T',
           '__FD_SETSIZE', '_SYS_TYPES_H', 'N14pthread_cond_t3DOT_7E',
           '__USE_XOPEN2K8', '__time_t_defined', '_SYS_CDEFS_H',
           '__intptr_t', '__u_long', '__timespec_defined',
           '__USE_GNU', 'ushort', '__USE_BSD', '_SIGSET_NWORDS',
           '__blkcnt_t', 'pthread_t', 'clockid_t', 'bcu_error_t',
           'BCU_CTL_OK', 'caddr_t', 'uint', '__rlim64_t', 'ino_t',
           '_LARGEFILE_SOURCE', '_POSIX_C_SOURCE', '_SIGSET_H_types',
           'bcu_rx_prepacket', '__mode_t', 'off64_t',
           'N16pthread_rwlock_t4DOT_10E', '__blksize_t', '__USE_SVID',
           'pthread_spinlock_t', '__SIZEOF_PTHREAD_MUTEX_T',
           '__USE_UNIX98', 'BCU_REG_WRITE', '__USE_ANSI', 'blkcnt_t',
           '__USE_MISC', 'fsblkcnt_t', 'select',
           '__BIT_TYPES_DEFINED__', 'u_quad_t', 'timespec',
           'register_t', 'BCU_ENCODE_NO_SUCH_COMMAND_ERR',
           'fsfilcnt64_t', '__daddr_t', 'ino64_t', 'bcu_tx_packet_t',
           '__sig_atomic_t', '_ENDIAN_H', 'fsblkcnt64_t',
           'BCU_REG_READ', '__uint8_t', '__PTHREAD_MUTEX_HAVE_PREV',
           '__USE_FORTIFY_LEVEL', '__SIZEOF_PTHREAD_RWLOCK_T',
           '_BITS_BYTESWAP_H', 'u_int', '__caddr_t', 'u_int8_t',
           '__blkcnt64_t', '__dev_t', '__STDC_ISO_10646__',
           'BCU_DECODE_NO_EOT_ERR', 'BCU_CTL_ERROR_MASK',
           '__STDC_IEC_559_COMPLEX__', '_SYS_SYSMACROS_H',
           '__USE_XOPEN_EXTENDED', '__suseconds_t', 'pid_t',
           'timer_t', 'quad_t', 'u_long', '__clock_t_defined',
           '__USE_LARGEFILE', 'bcu_rx_header_t',
           '__SIZEOF_PTHREAD_COND_T', '_FEATURES_H', 'useconds_t',
           '__socklen_t', 'pthread_key_t',
           'BCU_ENCODE_BUF_OVERRUN_ERR', 'bcu_error', 'loff_t',
           '__USE_POSIX199506', '_BITS_TYPES_H', '__BIG_ENDIAN',
           '__u_char', 'BCU_ENCODE_BUF_NOT_EMPTY_ERR',
           '__WORDSIZE_COMPAT32', 'pselect', 'int64_t',
           '__fsblkcnt_t', '__rlim_t', 'time_t', '__pthread_list_t',
           'blkcnt64_t', 'nlink_t', 'BCU_DECODE_ERROR_MASK',
           '_XOPEN_SOURCE_EXTENDED', '__quad_t', 'timeval',
           '__USE_XOPEN2K8XSI', '__SIZEOF_PTHREAD_RWLOCKATTR_T',
           '__PDP_ENDIAN', '__u_short', '__fsblkcnt64_t',
           'bcu_tx_header_t', '__key_t', 'fsid_t',
           '_LARGEFILE64_SOURCE', '_XOPEN_SOURCE', 'u_short',
           '__SIZEOF_PTHREAD_ATTR_T', 'key_t', '__USE_ISOC95',
           '__ino_t', 'int8_t', 'BCU_CTL_LEN_ERR', '__GLIBC__',
           'bcu_tx_packet', '__USE_ISOC99', '__fsfilcnt_t', '__u_int',
           'suseconds_t', '__STDC_IEC_559__', '__int64_t',
           '_BSD_SOURCE', 'ssize_t', '__clock_t', 'dev_t', '__uid_t',
           '__int8_t', '__FD_ZERO_STOS']
