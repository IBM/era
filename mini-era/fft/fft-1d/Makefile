#INPUT_SIZE = SMALL
#INPUT_SIZE = MEDIUM
INPUT_SIZE = LARGE

CC = gcc

CFLAGS = -pedantic -Wall -O0 -g -DINPUT_SIZE=INPUT_SIZE_$(INPUT_SIZE)
INCLUDES =  
LFLAGS = 
LIBS = -lm

TARGET = main
OBJECTS = fft.o octave.o timer.o 

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) $(CFLAGS) $(INCLUDES) -o $(TARGET).exe $(TARGET).c $(LFLAGS) $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	$(RM) $(TARGET).exe $(OBJECTS) *.mat
