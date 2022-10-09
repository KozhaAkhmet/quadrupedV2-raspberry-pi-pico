#! /bin/bash

sudo rm -r build/
mkdir "build"
cd build/
cmake ..
make -j4
sudo mv QuadrupedV2.uf2 /media/sk8/RPI-RP21
cd ..
