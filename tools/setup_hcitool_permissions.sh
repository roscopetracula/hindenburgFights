#!/bin/bash

# Set capabilities on hcitool so that non-root users can use it to
# scan, etc.  Note that this effect is system-wide.
sudo setcap 'cap_net_raw,cap_net_admin+eip' `which hcitool`
getcap `which hcitool`

