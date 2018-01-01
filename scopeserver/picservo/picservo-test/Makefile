#
# picservo-test Makefile for Linux systems using libnmc
#

CXXFLAGS = -g -c -O2 -Wall -I../libnmc
LIBS = -lstdc++ -lrt -lnmc

####
PROGS=picservo-test
OBJS=picservo-test.o
SILENT = @
#######

all: $(PROGS)

picservo-test: $(OBJS)
	$(SILENT) gcc $(OBJS) $(LIBS) -o picservo-test

picservo-test.o: picservo-test.cpp

clean:
	$(RM) ${OBJS} picservo-test

distclean: clean
	rm -f ../picservo-test.tar.gz

dist: distclean
	cd ..; \
	tar -cvf picservo-test.tar picservo-test; \
	gzip picservo-test.tar
