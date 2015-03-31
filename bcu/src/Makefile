ARCH=$(shell uname -m)
#ARCH=$(shell arch) deprecated

CFLAGS=-fPIC -Wall -g

INCDIR=../include
LIBDIR=../lib
OBJDIR=.obj
PYBINDINGDIR=../pybindings

INSTDIR=/my-inst-path
INSTINCDIR=$(INSTDIR)/include
INSTLIBDIR=$(INSTDIR)/lib
INSTPYBINDINGDIR=$(INSTDIR)/pymodules/bcu
MACUSBINC=../../driver

LIBBCU=libbcu.so

ifeq ($(ARCH),x86_64)
	LFLAGS=-fPIC -m64 -shared -Wl -lrt
	INSTLIBDIR=$(INSTDIR)/lib64
else
	LFLAGS=-fPIC -shared -Wl -lrt

endif

H2XML=h2xml.py
XML2PY=xml2py.py 

#H2XMLFLAGS=-q -k
H2XMLFLAGS=-c

#XML2PYFLAGS=-ksf
#XML2PYFLAGS=-kdefst -d
XML2PYFLAGS=-kdefst
#XML2PYFLAGS=-kdest

INSTINCS=-I$(INSTINCDIR)

INCS=-I$(MACUSBINC) -I$(INCDIR) -I$(INSTINCDIR)

HEADERS=$(INCDIR)/bcu/bcu_reg.h\
	$(INCDIR)/bcu/bcu_le98.h

OBJS=$(OBJDIR)/bcu_le98.o

BCUBINDING=$(PYBINDINGDIR)/bcu_reg.py\
	$(PYBINDINGDIR)/bcu_le98.py


all: $(HEADERS) $(OBJS) $(LIBDIR)/libbcu.so

inst: all
	su -c "cp -a $(INCDIR)/bcu $(INSTINCDIR)/ && cp -a $(LIBDIR)/$(LIBBCU) $(INSTLIBDIR)/"

# all doesn't generate bindings, use py to do them
pybcu: $(LIBDIR)/$(LIBBCU) $(BCUBINDING)

# all doesn't generate bindings, use py to do them
py: pybcu

pyinst: py
	su -c "[ ! -d $(INSTPYBINDINGDIR) ] && mkdir -p $(INSTPYBINDINGDIR); cp -a $(BCUBINDING) $(INSTPYBINDINGDIR)/"

$(OBJDIR)/%.o: %.c
	$(CC) -c $(INCS) $(CFLAGS) $< -o $@

$(LIBDIR)/$(LIBBCU): $(HEADERS) $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o $@
#	$(CC) $(LFLAGS),$@ $(OBJS) -o $@

# bcu xml description
$(OBJDIR)/%.xml: $(INCDIR)/bcu/%.h
	$(H2XML) $(INCS) $(PWD)/$< -o $(PWD)/$@ $(H2XMLFLAGS)

# bcu python binding
$(PYBINDINGDIR)/%.py: $(OBJDIR)/%.xml
	$(XML2PY) $(PWD)/$< $(XML2PYFLAGS) -l $(LIBBCU) -o $(PWD)/$@

clean:
	rm -f $(OBJDIR)/*.o $(LIBDIR)/*.so *~

cleanall: clean
	rm -f $(PYBINDINGDIR)/*.py
