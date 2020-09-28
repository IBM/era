# Standalone-ERA: Demo Version of the Main ERA Workload

The Demo-ERA serves as a stand-alone version of the workload for Colaborative Autonomous Vehicles,
intended to drive the EPOCHS full system design and development methodology through Phase-2 of the
DSSoC Program. The Demo-ERA will serve as the origination workload from which the EPOCHS Phase-2 SoC will be
developed through application of the full suite of EPOCHS Domain-Specific SoC tools.

This version of Demo-ERA runs as a standalone program, not requiring a separate autonomous vehicle
simulation infrastructure in order to drive the Deom-ERA program.  This version is driven by a
"trace" input (taken from a simulator run) that provides the necessary ERA inputs.  These inputs are
packaged in a "bagfile" and some example bagfiles can be made available (but they are quite
large and not included in the baseline github repository).

## System Requirements

Demo-ERA has been successfully built and executed using the following set-up:
 - Ubuntu 18.04
 - Python 2.7
 

## Installation and Build

Installation is a simple matter of cloning the repository, and then
compiling the code using cmake.

```
git clone https://github.com/IBM/era/tree/standalone_era
cd era
mkdir build
cd build
cmake ..
make
```

At this point, you should have generated two executables file named ```eraX```
which are the main ERA executables for this Demo-ERA version.  There are two
executables because one is generated for each of the Autonomous Vehicles (cars)
in the simulation.  The bagfile contains data currently for two AVs, so we generate the
two ERA executables:
 - era1 : the ```hero1``` executable, that reads and reacts to the hero1 bagfile contents
 - era2 : the ```hero2``` executable, that reads and reacts to the hero2 bagfile contents

## Execution

To execute the Demo-ERA program requires two simultaneous processes, one to read the bagfile,
which it then streams through a TCP/IP socket to the main Demo-ERA program.  Meanwhile the main
Demo-ERA program, running in another process, watches the socket for input data.

The easiest way to run the standalone Demo-ERA is to start multiple terminals in pairs, and in one
to invoke the bagfile reader/publisher and then in the other to start the associated
```eraX``` executable.  
To do this, first obtain a bagfile, which we will assume is named ```bagfile``` and stored in
```era/data```.  Activate one terminal, and type

```
cd era/src
python ./read_bagfile_1.py ../data/bagfile
```

Now activate a second terminal, and type
```
cd era/build
./era1
```

At this point, you will have invoked the standalone Demo-ERA program for AV 1
and driven it with your bagfile.  This is similarly done for AV 2, using the
similar commands but with the idnetifier 2 in place of 1.



### Usage

Currently, the standalone Demo-ERA program does not support command-line options, etc.
As such, there is very little to the usage at this point -- please refer to the ```Execution```
section above.


## Contacts and Current Maintainers

 - Akin Sisbot (easisbot@us.ibm.com)
 - J-D Wellman (wellman@us.ibm.com)
 - Augusto Vega (ajvega@us.ibm.com)
 