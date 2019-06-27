CC=gcc
CFLAGS=-lm -w -g
SOURCES= main.c vector.c Preferences.c NeuralNetwork.c MNistDoc.c
EXECUTABLE=main
all: $(SOURCES)
	$(CC) -o $(EXECUTABLE) $(SOURCES) $(CFLAGS)

clean:
	rm $(EXECUTABLE)
