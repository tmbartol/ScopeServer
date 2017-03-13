#
# libnmc Makefile for Linux systems
#

CXXFLAGS = -fPIC -g -c -O2 -Wall
LIBS = -lc
LIBDIR = /usr/local/lib

####
PROGS=libnmc install
OBJS=sio_util.o nmccom.o picio.o picservo.o picstep.o
SILENT = @
#######

all: $(PROGS)

libnmc: $(OBJS)
	$(SILENT) gcc -shared -Wl,-soname,libnmc.so.1 $(OBJS) $(LIBS) \
          -o libnmc.so.1.0

sio_util.o: sio_util.cpp sio_util.h

nmccom.o: nmccom.cpp nmccom.h sio_util.h

picio.o: picio.cpp picio.h nmccom.h sio_util.h

picservo.o: picservo.cpp picservo.h nmccom.h sio_util.h

picstep.o: picstep.cpp picstep.h


clean:
	$(RM) ${OBJS} libnmc.so.1.0 $(LIBDIR)/libnmc.so.1.0 \
          $(LIBDIR)/libnmc.so.1 $(LIBDIR)/libnmc.so


mostlyclean:
	$(RM) ${OBJS} libnmc.so.1.0


distclean: mostlyclean
	rm ../libnmc.tar.gz

install:
	install -m 755 -o root libnmc.so.1.0 $(LIBDIR)
	$(SILENT) cd $(LIBDIR) ; \
	  ln -s libnmc.so.1.0 libnmc.so.1 ; \
	  ln -s libnmc.so.1 libnmc.so
	$(SILENT) echo "############################"
	$(SILENT) echo "#"
	$(SILENT) echo "# Make sure $(LIBDIR) is in your ld.so.conf, then run ldconfig."
	$(SILENT) echo "#"
	$(SILENT) echo "############################"

dist: distclean
	cd ..; \
	tar -cvf libnmc.tar libnmc; \
	gzip libnmc.tar
