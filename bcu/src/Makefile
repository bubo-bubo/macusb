ARCH=$(shell uname -m)
#ARCH=$(shell arch) deprecated

CFLAGS=-fPIC -Wall -g

INCDIR=../include
LIBDIR=../lib
OBJDIR=.obj

INSTDIR=/my-inst-path
INSTINCDIR=$(INSTDIR)/include
INSTLIBDIR=$(INSTDIR)/lib
MACUSBINC=../../driver

LIBBCU=libbcu.so

ifeq ($(ARCH),x86_64)
	LFLAGS=-fPIC -m64 -shared -lrt
	INSTLIBDIR=$(INSTDIR)/lib64
else
	LFLAGS=-fPIC -shared -lrt

endif

INSTINCS=-I$(INSTINCDIR)

INCS=-I$(MACUSBINC) -I$(INCDIR) -I$(INSTINCDIR)

HEADERS=$(INCDIR)/bcu/bcu_reg.h \
	$(INCDIR)/bcu/bcu_dev.h \
	$(INCDIR)/bcu/bcu_le99s.h \
	$(INCDIR)/bcu/bcu_le98.h \
	$(INCDIR)/bcu/bcu_em11.h

OBJS=$(OBJDIR)/bcu_dev.o $(OBJDIR)/bcu_le99s.o $(OBJDIR)/bcu_le98.o $(OBJDIR)/bcu_em11.o

all: $(HEADERS) $(OBJS) $(LIBDIR)/libbcu.so

inst: all
	su -c "cp -a $(INCDIR)/bcu $(INSTINCDIR)/ && cp -a $(LIBDIR)/$(LIBBCU) $(INSTLIBDIR)/"


$(OBJDIR)/%.o: %.c
	$(CC) -c $(INCS) $(CFLAGS) $< -o $@

$(LIBDIR)/$(LIBBCU): $(HEADERS) $(OBJS)
	$(CC) $(LFLAGS) -o $@ $(OBJS)
#	$(CC) $(LFLAGS),$@ -o $@ $(OBJS)

clean:
	rm -f $(OBJDIR)/*.o $(LIBDIR)/*.so *~
