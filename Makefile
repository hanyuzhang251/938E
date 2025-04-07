all: build

build:
	@make -f Makefile2 -j $(nproc)

clean:
	@make -f Makefile2 clean

# This Makefile is a wrapper for Makefile2. It provides a simple interface to build and clean the project.
# It includes the common.mk file for shared settings and configurations.
# The all target builds the project by calling Makefile2, while the clean target cleans the project.