MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00040000
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00007F00 /* Hack to protect 256B MSP 0x00008000 32 KB RAM */
}

SECTIONS
{
  /* ## Vector Table in RAM */
  .vector_ram (NOLOAD) : ALIGN(1024)
  {
    __vector_ram_start = .;
    KEEP(*(.vector_ram));
    __vector_ram_end = .;
  } >RAM
}
