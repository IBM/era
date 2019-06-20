CC = gcc

CFLAGS = -pedantic -Wall -O0 -g
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
