/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x100000
  EXTFLASH (rx) : ORIGIN = 0x12000000, LENGTH = 0x8000000
  /*RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x40000*/
  /*limit ram to RAM0 and RAM1*/
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x4000
  CODE_RAM (rwx) : ORIGIN = 0x800000, LENGTH = 0x40000
}


INCLUDE "nrf_common.ld"
