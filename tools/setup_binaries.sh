#!/bin/bash

DIRNAME="`dirname $0`"

# Set capabilities on hcitool.
"$DIRNAME/setup_hcitool_permissions.sh"

# Build bluepy.
"$DIRNAME/setup_bluepy.sh"
