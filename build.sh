make clean
time make -j$(($(sysctl -n hw.ncpu) + 1))
