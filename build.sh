make clean
time make -j$(($(sysctl -n hw.ncpu) + 1))
echo "compiled on $(date '+%Y-%m-%d %H:%M:%S')"