# Mini-ERA FFT Kernel

## Requirements

The provided FFTW library source code **has to be compiled first**:

``` 
cd fftw-3.3.8/
./configure --prefix=`pwd` --disable-fortran --enable-float CFLAGS=-g
make -j8
make install
```

## Installing FFT

```
make
```

Upon successful completion, the file `main.exe` is generated. This is a simple example that computes a complex 1D FFT of size 64, 1000 times:

```
./main.exe 64 1000
Computing 1000 FFTs of size 64...
Time elpased is 0.003048543 seconds
```

