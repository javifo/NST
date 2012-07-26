#!/bin/bash

#
# Build U-Boot image when `mkimage' tool is available.
#

MKIMAGE=$(type -path "${CROSS_COMPILE}mkimage")

if [ -z "${MKIMAGE}" ]; then
	MKIMAGE=$(type -path mkimage)
	if [ -z "${MKIMAGE}" ]; then
		# Add mkimage to path if not yet there. Note that kernel and u-boot are sibling directories in B&N directory structure
	    export PATH=$PATH:$PWD/../u-boot/tools
	    MKIMAGE=$(type -path mkimage)
	    if [ -z "${MKIMAGE}" ]; then
		# Doesn't exist
		echo '"mkimage" command not found - U-Boot images will not be built' >&2
		echo "Maybe you haven't built u-boot yet. Change to u-boot directory and build it" >&2
		exit 0;
		else
		echo '"mkimage" not in path. Executable has been found and added to path. Continuing with the build...' >&2
		fi
	fi
fi

# Call "mkimage" to create U-Boot image
${MKIMAGE} "$@"
