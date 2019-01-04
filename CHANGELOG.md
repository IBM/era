# Changelog

## [1.1] - 2018-11-02
### Added
- Start using this Changelog file to keep track of changes between ERA versions.
- Map fusion module to compare and fuse local and remote 2D occupancy maps.
- Profiling support for ROS nodes and GNU Radio blocks using Linux perf.

### Changed
- The `gui:=false` option at launch time now disables RViz's GUI too (in addition to Gazebo's GUI).
- Robots connect to each other through the DSRC module (before, each robot independently talked to a remote "dummy" agent).
- "ROS Interface" GNU Radio custom block ported from Python to C++, using LZ4 instead of bzip2 for faster compression/decompression and simpler programming.

### Removed
- DSRC launch file (DSRC package).
- Map producer/consumer (DSRC package).
- Remote agent (DSRC package).
- Wireshark wrapper (DSRC package).

### Fixed
- 