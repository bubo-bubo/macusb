CC      = gcc
CPP	= g++
LD      = gcc
LDPP	= g++
DRVDIR	= ../driver
BCUDIR	= ../bcu

CFLAGS    = -Wall -Wstrict-prototypes
#CFLAGS   += -g -O3
CFLAGS   += -g -O0
CFLAGS   += -I$(DRVDIR) -I$(BCUDIR)/include
#CFLAGS   += -O2

LDFLAGS  = -lpopt
#LDFLAGS += -lefence -lm

OBJDIR  = .obj
PROGS32	= ioctl32
PROGS	= cmd ioctl

all: $(PROGS)

32: $(PROGS32)

$(OBJDIR)/%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJDIR)/%32.o: %.c
	$(CC) -m32 -c $(CFLAGS) $< -o $@

$(OBJDIR)/%.o: %.cc
	$(CPP) -c $(CPPFLAGS) $< -o $@

$(OBJDIR)/%32.o: %.cc
	$(CPP) -m32 -c $(CPPFLAGS) $< -o $@

ioctl: $(OBJDIR)/ioctl.o
	$(LD) -o ioctl $(LDFLAGS) $(OBJDIR)/ioctl.o

ioctl32: $(OBJDIR)/ioctl32.o
	$(LD) -m32 -o ioctl32 $(LDFLAGS) $(OBJDIR)/ioctl32.o

cmd: $(OBJDIR)/cmd.o
	$(LD) -o cmd $(LDFLAGS) -L$(BCUDIR)/lib -lbcu $(OBJDIR)/cmd.o

cmd32: $(OBJDIR)/cmd32.o
	$(LD) -m32 -o cmd32 $(LDFLAGS) -L$(BCUDIR)/lib -lbcu $(OBJDIR)/cmd32.o


clean:
	rm -f $(OBJDIR)/*.o *~

cleanall: clean
	rm -f $(PROGS) $(PROGS32)
