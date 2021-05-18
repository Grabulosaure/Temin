CC=gcc
#CFLAGS=-W -Wall -ansi -pedantic
CFLAGS=-O4
LDFLAGS=
EXEC=temin

all: $(EXEC)

temin: main.o
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm -rf *.o

