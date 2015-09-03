#!/bin/bash
if [ "$USER" != "root" ]; then
    echo Rerunning as root.
    exec sudo $0 $*
fi

while ( true ); do
    xboxdrv -v -d --dbus session --daemon --led 2 --mimic-xpad --next-controller --led 3 --mimic-xpad --next-controller --led 4 --mimic-xpad --next-controller --led 5 --mimic-xpad --next-controller --led 2 --mimic-xpad --next-controller --led 3 --mimic-xpad --next-controller --led 4 --mimic-xpad --next-controller --led 5 --mimic-xpad --next-controller --led 2 --mimic-xpad --next-controller --led 3 --mimic-xpad --next-controller --led 4 --mimic-xpad --next-controller --led 5 --mimic-xpad
    echo Sleeping 5 seconds before rerunning...
    sleep 5
done

