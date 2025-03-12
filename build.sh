make clean
CC="ccache clang"
CXX="ccache clang++"
time make -j$(($(sysctl -n hw.ncpu) + 1))