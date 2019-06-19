# FFTW Kernels

## Requirements

The provided FFTW library source code has to be compiled first:

``` 
cd fftw-3.3.8/
./configure --prefix=`pwd` --disable-fortran --enable-float
make -j8
make install
```

