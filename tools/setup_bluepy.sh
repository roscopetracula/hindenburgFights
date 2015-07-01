#!/bin/bash

DIRNAME="`dirname $0`"

# Build bluepy.
pushd "$DIRNAME/../bluepy/bluepy"
make clean && make
popd

