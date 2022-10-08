#! /bin/bash

sudo rm -r build/
mkdir "build"
cd build/
cmake ..
make -j4
sleep 2
sudo mv QuadrupedV2.uf2 /media/sk8/RPI-RP2
cd ..
