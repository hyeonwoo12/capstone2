#! /bin/sh

if command -v gdb-multiarch &> /dev/null
then
	GDB_CMD=$(command -v gdb-multiarch)

elif command -v arm-none-eabi-gdb &> /dev/null
then
	GDB_CMD=$(command -v arm-none-eabi-gdb)

else
	echo "gdb arm-none-eabi or multi-arch not found"
	exit 1
fi

/usr/bin/JLinkGDBServerExe -device MK66FN2M0xxx18 -select usb -endian little -if SWD -speed auto -ir -LocalhostOnly 1 -strict -vd -singlerun &

cd /home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default && ${GDB_CMD} -silent -nh \
	-iex "set auto-load safe-path /home/hwpark/capstone2/px4-firmware/build/nxp_fmuk66-v3_default" \
	-ix=/home/hwpark/capstone2/px4-firmware/platforms/nuttx/NuttX/nuttx/tools/nuttx-gdbinit \
	-ex "target remote localhost:2331" \
	-ex "monitor reset 0" \
	-ex "load" \
	-ex "monitor reset 0" \
	-ex "continue" \
	nxp_fmuk66-v3_default.elf

# exit with status of last command
exit $?
