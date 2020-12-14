#!/bin/bash

# Save the current configuration file
mv .config .config.save

echo "Building 800-size executables..."
sed 's/GRID_MAP_X_DIM=100/GRID_MAP_X_DIM=800/' .config.save | sed 's/GRID_MAP_Y_DIM=100/GRID_MAP_Y_DIM=800/' | sed 's/RAYTR_RANGE=100/RAYTR_RANGE=800/' > .config
make clean; make
#mv era1.exe era1c_800.exe
#mv era2.exe era2c_800.exe

echo "Building 700-size executables..."
sed 's/GRID_MAP_X_DIM=100/GRID_MAP_X_DIM=700/' .config.save | sed 's/GRID_MAP_Y_DIM=100/GRID_MAP_Y_DIM=700/' | sed 's/RAYTR_RANGE=100/RAYTR_RANGE=700/' > .config
make clean; make
#mv era1.exe era1c_700.exe
#mv era2.exe era2c_700.exe

echo "Building 600-size executables..."
sed 's/GRID_MAP_X_DIM=100/GRID_MAP_X_DIM=600/' .config.save | sed 's/GRID_MAP_Y_DIM=100/GRID_MAP_Y_DIM=600/' | sed 's/RAYTR_RANGE=100/RAYTR_RANGE=600/' > .config
make clean; make
#mv era1.exe era1c_600.exe
#mv era2.exe era2c_600.exe

echo "Building 500-size executables..."
sed 's/GRID_MAP_X_DIM=100/GRID_MAP_X_DIM=500/' .config.save | sed 's/GRID_MAP_Y_DIM=100/GRID_MAP_Y_DIM=500/' | sed 's/RAYTR_RANGE=100/RAYTR_RANGE=500/' > .config
make clean; make
#mv era1.exe era1c_500.exe
#mv era2.exe era2c_500.exe

echo "Building 400-size executables..."
sed 's/GRID_MAP_X_DIM=100/GRID_MAP_X_DIM=400/' .config.save | sed 's/GRID_MAP_Y_DIM=100/GRID_MAP_Y_DIM=400/' | sed 's/RAYTR_RANGE=100/RAYTR_RANGE=400/' > .config
make clean; make
#mv era1.exe era1c_400.exe
#mv era2.exe era2c_400.exe

echo "Building 300-size executables..."
sed 's/GRID_MAP_X_DIM=100/GRID_MAP_X_DIM=300/' .config.save | sed 's/GRID_MAP_Y_DIM=100/GRID_MAP_Y_DIM=300/' | sed 's/RAYTR_RANGE=100/RAYTR_RANGE=300/' > .config
make clean; make
#mv era1.exe era1c_300.exe
#mv era2.exe era2c_300.exe

echo "Building 200-size executables..."
sed 's/GRID_MAP_X_DIM=100/GRID_MAP_X_DIM=200/' .config.save | sed 's/GRID_MAP_Y_DIM=100/GRID_MAP_Y_DIM=200/' | sed 's/RAYTR_RANGE=100/RAYTR_RANGE=200/' > .config
make clean; make
#mv era1.exe era1c_200.exe
#mv era2.exe era2c_200.exe

echo "Building 100-size executables..."
sed 's/GRID_MAP_X_DIM=100/GRID_MAP_X_DIM=100/' .config.save | sed 's/GRID_MAP_Y_DIM=100/GRID_MAP_Y_DIM=100/' | sed 's/RAYTR_RANGE=100/RAYTR_RANGE=100/' > .config
make clean; make
#mv era1.exe era1c_100.exe
#mv era2.exe era2c_100.exe

# Restore the original cofiguration file
mv .config.save .config
