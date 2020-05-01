/* Memory layout for stm32f427 */
MEMORY
{
  /* NOTE K = 1024 bytes */
  /* FLASH and RAM are mandatory memory regions */

  FLASH  : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM    : ORIGIN = 0x20000000, LENGTH = 256K

}

/*
This is where the call stack will be allocated.
The stack is of the full descending type.
Place the stack at the end of RAM.
*/
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* The location of the .text section can be overridden using the
   `_stext` symbol.  By default it will place after .vector_table */
/* _stext = ORIGIN(FLASH) + 0x40c; */

