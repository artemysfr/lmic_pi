CFLAGS=-I../../lmic
LDFLAGS=-lwiringPi -lpthread

EXEC=udp-wait-and-send

${EXEC}: udp-wait-and-send.cpp
	cd ../../lmic && $(MAKE)
	$(CC) $(CFLAGS) -o $@ $^ ../../lmic/*.o $(LDFLAGS)

all: ${EXEC}

.PHONY: clean

clean:
	rm -f *.o ${EXEC}
