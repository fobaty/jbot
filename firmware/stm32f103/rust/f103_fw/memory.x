MEMORY
{
  /* The top 4 KB holds the two NVM parameter pages and is never linked into
   * the firmware image. See hal::nvm_restore / hal::nvm_save. */
  FLASH : ORIGIN = 0x08000000, LENGTH = 252K
  RAM   : ORIGIN = 0x20000000, LENGTH = 48K
}
