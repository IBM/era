# Radar Processing

## Closest object FMCW

### Python Files
* `./generate_fmcw_waveform.py -d 100` generates a datafile `temp.dat` for an object at distance 100 meters
* `./calculate_dist_from_fmcw.py temp.dat -m` calculates the distance to the closest object

### C implementation
* `make`
* `./calculate_dist_from_fmcw.exe` reads `temp.dat` and calculates distance to the closest object

## Multiple object FMCW

### Python Files
* `./generate_fmcw_waveform.py -d 10 25 100 200` generates a datafile `temp.dat` for multiple objects at the specified distances
* `./calculate_dist_from_fmcw.py temp.dat` calculates the distances of all the objects

### C implementation
* Still to be implemented
