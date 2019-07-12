# GNURadio Viterbi test for different configurations:

This code is separated from the GNURadio as a stand-alone kernel for use in our ongoing analyses.

# Requirements
This code is relatively simple C, and requires only

    GCC supporting c99
    make
# Installation and Usage
Clone Viterbi either as part of the full Mini-ERA or from my local space:

    git clone git@github.ibm.com:varunmannamibm/viterbi_test.git

# Use Cases (Examples) Implementation
To build the library:

    cd viterbi_test
    make

To make the viterbi_test program:

    cd viterbi_test
    make viterbi_test

To run the test program:

    ./viterbi_test Filename #this is the configuration file in txt format
