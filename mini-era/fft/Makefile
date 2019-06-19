CC = gcc

CFLAGS = -pedantic -Wall -O0
INCLUDES = -I`pwd`/fftw-3.3.8/include 
LFLAGS = -L`pwd`/fftw-3.3.8/lib
LIBS = -lfftw3 -lfftw3f -lm

TARGET = main

all: $(TARGET)

$(TARGET): $(TARGET).c
	$(CC) $(CFLAGS) $(INCLUDES) -o $(TARGET).exe $(TARGET).c $(LFLAGS) $(LIBS)

clean:
	$(RM) $(TARGET).exe
