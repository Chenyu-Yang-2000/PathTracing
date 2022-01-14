#!/usr/bin/env bash

# If project not ready, generate cmake file.
if [[ ! -d build ]]; then
    echo "good"
else
    rm -rf build
fi
mkdir -p build
cd build
cmake ..
make -j
cd ..

# Run all testcases. 
# You can comment some lines to disable the run of specific examples.
mkdir -p output
bin/PT raytracing testcases/scene05_tank.txt output/scene05_t.bmp
bin/PT pathtracing testcases/scene05_tp.txt output/scene05_tp.bmp
bin/PT pathtracing testcases/scene11_m3.txt output/scene11_m3.bmp
bin/PT pathtracing testcases/scene11_m5.txt output/scene11_m5.bmp