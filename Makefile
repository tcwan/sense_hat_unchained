CC=aarch64-linux-gnu-gcc
AR=aarch64-linux-gnu-ar
CFLAGS=-g -c -Wall -O2 -I.
LIBS= -L. -lsensehat -lpthread -lm

all: sensedemo

sensedemo: libsensehat.a main.o
	$(CC) main.o $(LIBS) -o sensedemo

main.o: main.c
	$(CC) $(CFLAGS) main.c

libsensehat.a: sensehat.o
	$(AR) -rc libsensehat.a sensehat.o

sensehat.o: sensehat.h sensehat.c
	$(CC) $(CFLAGS) sensehat.c

clean:
	rm -f *.o sensedemo libsensehat.a

