#!/bin/bash

# Save the current configuration file
mv .config .config.save

echo "Building All-Software..."
sed 's/##/#/g' .config.save | sed 's/##/#/g' | sed 's/##/#/g' | sed 's/CONFIG_FFT_EN=y/#CONFIG_FFT_EN=y/' | sed 's/CONFIG_RECV_FFT_EN=y/#CONFIG_RECV_FFT_EN=y/' | sed 's/CONFIG_XMIT_FFT_EN=y/#CONFIG_XMIT_FFT_EN=y/' | sed 's/CONFIG_VITERBI_EN=y/#CONFIG_VITERBI_EN=y/' > .config
make clean; make xmit_pipe recv_pipe t_era1 t_era2 t_pera1 t_pera2
mv .config saved.config.SW

echo "Building Hardware Viterbi..."
sed 's/##/#/g' .config.save | sed 's/##/#/g' | sed 's/##/#/g' | sed 's/CONFIG_FFT_EN=y/#CONFIG_FFT_EN=y/' | sed 's/CONFIG_RECV_FFT_EN=y/#CONFIG_RECV_FFT_EN=y/' | sed 's/CONFIG_XMIT_FFT_EN=y/#CONFIG_XMIT_FFT_EN=y/' | sed 's/#CONFIG_VITERBI_EN=y/CONFIG_VITERBI_EN=y/' | sed 's/CONFIG_HW_SM_DECODE=false/CONFIG_HW_SM_DECODE=true/' > .config
make clean; make xmit_pipe recv_pipe t_era1 t_era2 t_pera1 t_pera2
cp .config saved.config.VHW

echo "Building Hardware FFT..."
sed 's/##/#/g' .config.save | sed 's/##/#/g' | sed 's/##/#/g' | sed 's/#CONFIG_FFT_EN=y/CONFIG_FFT_EN=y/' | sed 's/#CONFIG_RECV_FFT_EN=y/CONFIG_RECV_FFT_EN=y/' | sed 's/#CONFIG_XMIT_FFT_EN=y/CONFIG_XMIT_FFT_EN=y/' | sed 's/CONFIG_VITERBI_EN=y/#CONFIG_VITERBI_EN=y/' | sed 's/CONFIG_HW_SM_DECODE=false/CONFIG_HW_SM_DECODE=true/' > .config
make clean; make xmit_pipe recv_pipe t_era1 t_era2 t_pera1 t_pera2
cp .config saved.config.FHW

echo "Building Hardware All-Hardware..."
sed 's/##/#/g' .config.save | sed 's/##/#/g' | sed 's/##/#/g' | sed 's/#CONFIG_FFT_EN=y/CONFIG_FFT_EN=y/' | sed 's/#CONFIG_RECV_FFT_EN=y/CONFIG_RECV_FFT_EN=y/' | sed 's/#CONFIG_XMIT_FFT_EN=y/CONFIG_XMIT_FFT_EN=y/' | sed 's/#CONFIG_VITERBI_EN=y/CONFIG_VITERBI_EN=y/' | sed 's/CONFIG_HW_SM_DECODE=false/CONFIG_HW_SM_DECODE=true/' > .config
make clean; make xmit_pipe recv_pipe t_era1 t_era2 t_pera1 t_pera2
cp .config saved.config.allHW

# Now the Small-Decode = FALSE versions...
echo "Building Hardware Viterbi..."
sed 's/##/#/g' .config.save | sed 's/##/#/g' | sed 's/##/#/g' | sed 's/CONFIG_FFT_EN=y/#CONFIG_FFT_EN=y/' | sed 's/CONFIG_RECV_FFT_EN=y/#CONFIG_RECV_FFT_EN=y/' | sed 's/CONFIG_XMIT_FFT_EN=y/#CONFIG_XMIT_FFT_EN=y/' | sed 's/#CONFIG_VITERBI_EN=y/CONFIG_VITERBI_EN=y/' | sed 's/CONFIG_HW_SM_DECODE=true/CONFIG_HW_SM_DECODE=false/' > .config
make clean; make xmit_pipe recv_pipe t_era1 t_era2 t_pera1 t_pera2
cp .config saved.config.VHW

echo "Building Hardware FFT..."
sed 's/##/#/g' .config.save | sed 's/##/#/g' | sed 's/##/#/g' | sed 's/#CONFIG_FFT_EN=y/CONFIG_FFT_EN=y/' | sed 's/#CONFIG_RECV_FFT_EN=y/CONFIG_RECV_FFT_EN=y/' | sed 's/#CONFIG_XMIT_FFT_EN=y/CONFIG_XMIT_FFT_EN=y/' | sed 's/CONFIG_VITERBI_EN=y/#CONFIG_VITERBI_EN=y/' | sed 's/CONFIG_HW_SM_DECODE=true/CONFIG_HW_SM_DECODE=false/' > .config
make clean; make xmit_pipe recv_pipe t_era1 t_era2 t_pera1 t_pera2
cp .config saved.config.FHW

echo "Building Hardware All-Hardware..."
sed 's/##/#/g' .config.save | sed 's/##/#/g' | sed 's/##/#/g' | sed 's/#CONFIG_FFT_EN=y/CONFIG_FFT_EN=y/' | sed 's/#CONFIG_RECV_FFT_EN=y/CONFIG_RECV_FFT_EN=y/' | sed 's/#CONFIG_XMIT_FFT_EN=y/CONFIG_XMIT_FFT_EN=y/' | sed 's/#CONFIG_VITERBI_EN=y/CONFIG_VITERBI_EN=y/' | sed 's/CONFIG_HW_SM_DECODE=true/CONFIG_HW_SM_DECODE=false/' > .config
make clean; make xmit_pipe recv_pipe t_era1 t_era2 t_pera1 t_pera2
cp .config saved.config.allHW

# Restore the original cofiguration file
mv .config.save .config
