#! /bin/bash

cd build/
rm Quadruped*
cmake ..
make
mv QuadrupedV2.uf2 /media/sk8/RPI-RP2
cd ..
