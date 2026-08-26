MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  /* Reserve the final four 2 KiB pages in flash bank 2 for persistent data. */
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 248K
  RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}
