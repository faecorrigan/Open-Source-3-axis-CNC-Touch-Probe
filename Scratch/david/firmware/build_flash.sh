#!/bin/bash

C_FILE="${1%.*}.c"
ELF_FILE="${1%.*}.elf"
BIN_FILE="${1%.*}.bin"

if [ "$1" == "" ]
then
	echo "usage: ./build_flash.sh prog.c"
	exit 1
fi

set -ex

arm-none-eabi-gcc -mcpu=cortex-m4 -ffreestanding --static -nostartfiles -ggdb "$C_FILE" gcc_startup_nrf52840.S -T nrf52840_xxaa.ld -o "$ELF_FILE"
arm-none-eabi-objcopy -O binary "$ELF_FILE" "$BIN_FILE"
# openocd -f interface/stlink.cfg -f target/nrf52.cfg -c init -c targets -c 'reset init' -c "flash write_image erase ${BIN_FILE}" -c 'reset run'
arm-none-eabi-gdb ${ELF_FILE} \
	-ex "target extended-remote | openocd -f interface/stlink.cfg -f target/nrf52.cfg -c 'gdb_port pipe; log_output openocd.log'" \
	-ex "monitor reset halt" \
	-ex "load" \
	-ex "monitor flash write_image erase ${BIN_FILE}" \
	-ex "layout split" -ex "focus cmd" -ex "si"

