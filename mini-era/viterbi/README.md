# GNURadio Viterbi Function Stand-Alone

This code is separated from the <a href="https://github.com/gnuradio/gnuradio">GNURadio</a> as a stand-alone kernel for use in our ongoing analyses.

## Requirements

This code is relatively simple C, and requires only 

 - GCC supporting c99 
 - make 

## Installation and Usage

Clone the full Mini-ERA from github:

```
git clone https://github.com/IBM/era
```

The mini-era code resides in the mini-era folder:
```
cd era/mini-era/viterbi
```

## Usage Cases (Examples) Implementation

To build the libraries and test programs:
```
cd mini-era/viterbi
make all
```

This will build several library versions:
 - libviterbi.a : the general viterbi library 
 - libviterbiCk.a : the general viterbi library with run-time results checking for one built-in example
 - libviterbi_esp.a : the general viterbi library using a modified viterbi_butterfly2 function call interface
 - libviterbiCk.a : the general viterbi library using a modified viterbi_butterfly2 function call interface with run-time results checking for one built-in example

And several test programs:
 - vtest : Test run using built-in test data and run-time checking (i.e. libviterbiCk.a)
 - vtest_esp : Test run using built-in test data and run-time checking (i.e. libviterbiCk.a) with a modified viterbi_butterfly2 function call interface 
 - viterbi_test : Test run using input configuration file that includes input parms and run-time check data
 - viterbi_test_esp : Test run using input configuration file that includes input parms and run-time check data with a modified viterbi_butterfly2 function call interface 

To run the test programs with built-in data:
```
./vtest
./vtest_esp
```
To run the test programs with input configuration files:
```
./viterbi_test test_config/enc1.txt
./viterbi_test_esp test_config/enc1.txt
```
Note that there are 3 test configuration files at this time:
 - enc0.txt
 - enc1.txt
 - enc6.txt


To build another application using this viterbi, 
 - Make the library
 - Include library header (viterbi_standalone.h)
 - Compile your code and link with the viterbi library (-lviterbi)


For reference, the parameter installation, checking verification data, etc. are in viterbi_standalone.c

## Contacts, etc.

Maintainers: 
 - J-D Wellman : wellman@us.ibm.com
 - Varun Mannam : varunmanam@us.ibm.com

