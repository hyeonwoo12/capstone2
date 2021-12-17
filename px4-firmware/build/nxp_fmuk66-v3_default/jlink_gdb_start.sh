#! /bin/sh

killall JLinkGDBServerCLExe

/usr/bin/JLinkGDBServerCLExe -silent -device MK66FN2M0xxx18 -select usb -if SWD -speed auto -LocalhostOnly 1 -strict -vd -singlerun -timeout 3000 -powertarget 0 -nogui &
sleep 2

pgrep -i JLinkGDBServer

# exit with status of last command
exit $?
