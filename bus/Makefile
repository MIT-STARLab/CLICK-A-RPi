CC=cc
DEPS=crc.h
OBJ = businterface.o crc.o
LIBS=
CFLAGS=-I.

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

businterface: $(OBJ)
		$(CC) -o $@ $^ $(CFLAGS)
