/* Minimal, working GNU ld script for STM32F767ZI (2MB FLASH, 368KB SRAM1, 16KB SRAM2)
   - Places vectors/.text/.rodata in FLASH
   - .data runs in SRAM1, loaded from FLASH (single AT>FLASH)
   - .bss in SRAM1
   - Heap+stack reserved in SRAM1 (simple) */

ENTRY(Reset_Handler)

/* -------- Memory Regions (simple) -------- */
MEMORY
{
  RAM   (xrw) : ORIGIN = 0x20020000, LENGTH = 368K    /* SRAM1 */
  RAM2  (xrw) : ORIGIN = 0x2007C000, LENGTH = 16K     /* SRAM2 (unused here) */
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
}

/* Top of stack in RAM1 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Optional reserves */
_Min_Heap_Size  = 0x400;
_Min_Stack_Size = 0x800;

/* -------- Sections -------- */
SECTIONS
{
  /* Vector table in FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  /* Code in FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text*)                     /* code */
    *(.glue_7) *(.glue_7t)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
  } > FLASH

  /* Const data in FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata*)                   /* const data */
    *(.eh_frame*)
    . = ALIGN(4);
  } > FLASH

  /* Unwind tables in FLASH */
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  __exidx_start = .;
  .ARM.exidx : { *(.ARM.exidx*) } > FLASH
  __exidx_end = .;

  /* C/C++ init arrays in FLASH */
  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > FLASH

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } > FLASH

  /* Initialized data: run in RAM, load from FLASH
     -> single AT> FLASH (no AT(expr)). _sidata is auto set via LOADADDR. */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data*)
    *(.ramfunc*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT> FLASH

  /* Zero-initialized data in RAM */
  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /* Simple heap+stack reservation in RAM */
  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } > RAM

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
