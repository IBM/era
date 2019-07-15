# GNURadio Viterbi Function Stand-Alone

This code is separated from the <a href="https://github.com/gnuradio/gnuradio">GNURadio</a> as a stand-alone kernel for use in our ongoing analyses.

## Requirements

This code is relatively simple C, and requires only 

 - GCC supporting c99 
 - make 

## Installation and Usage

Clone Viterbi either as part of the full Mini-ERA or from my local space:

```
git clone git@github.ibm.com:wellman-us/my-mini-era.git
```

## Use Cases (Examples) Implementation

To build the library:
```
cd Viterbi
make lib
```
to make the test program:
``` 
cd Viterbi
make vtest
```

To run the test program:
```
./vtest
```

To build another application using this viterbi, 
 - Make the library
 - Include library header (viterbi_standalone.h)
 - Compile your code and link with the viterbi library (-lviterbi)


For reference, the parameter installation, checking verification data, etc. are in viterbi_standalone.c

## Contacts, etc.

Maintainers: 
 - J-D Wellman : wellman@us.ibm.com

