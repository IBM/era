CC=gcc
CFLAGS=-I.
SOURCES= main.c mnist_parse.c
DEPS= mnist_parse.h
OBJECTS=main.o mnist_parse.o
EXECUTABLE=main
all:
	$(CC) -o $(EXECUTABLE) $(SOURCES)
run:
	./main
clean:
	rm $(EXECUTABLE)
