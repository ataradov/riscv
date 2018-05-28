/**/
MEMORY
{
  mem  (rwx) : ORIGIN = 0x00000000, LENGTH = 0x2000 /* 8k */
}

__top_mem = ORIGIN(mem) + LENGTH(mem);

ENTRY(entry)

SECTIONS
{
  .text : ALIGN(4)
  {
    *(.text.entry)
    *(.text*)
    . = ALIGN(4);
  } > mem

  .rodata : ALIGN(4)
  {
    *(.rodata)
    *(.rodata.*)
    . = ALIGN(4);
  } > mem

  . = ALIGN(4);
  _etext = .;

  .data : ALIGN(4)
  {
    *(.data)
    *(.data.*)
    *(.sdata)
    *(.sdata.*)
    . = ALIGN(4);
  } > mem

  .bss : ALIGN(4)
  {
    _bss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    PROVIDE(_end = .);
  } > mem

  PROVIDE(_stack_top = __top_mem);
}

