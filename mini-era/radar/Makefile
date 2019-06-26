CC = gcc

CFLAGS = -pedantic -Wall -O0 -g 

all:
	cd ../fft/fft-1d && make
	gcc ../fft/fft-1d/fft.o -pedantic -Wall -O0 -g  calculate_dist_from_fmcw.c -L /mnt/hgfs/WinDownloads/DSSOC/era/mini-era/fft/fft-1d/ -lm -I ../fft/fft-1d -o calculate_dist_from_fmcw.exe

clean:
	*.exe *.dat
