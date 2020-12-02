PROJ=businterface
CC=cc
SRC=businterface.c
# LIBS=-lgpiod
LIBS=-pthread -lpigpiod_if2 -lrt
CFLAGS=-D CONSUMER=\"$(PROJ)\"

all:
	$(CC) $(SRC) $(LIBS) $(CFLAGS) -o $(PROJ)

clean:
	rm $(PROJ)
