@echo off
color f0
title AvrDude GUI Command Window
REM The files t85_no_pullup.hex and t85_entry_on_power_on_no_pullup.hex are identical!
@echo Upgrade Digispark Bootloader with spi programming by avrdude
@echo. Writing ATtiny85 Lfuse to Micronucleus mode: E1 (16 MHz PLL, 1024 Clocks from Power Down sleep + 64 ms after reset), Hfuse to DF (BOD disable) and EFuse to FE (self programming enabled) 
avrdude -pt85 -cstk500v1 -PCOM6 -b19200 -u -Uflash:w:t85_entry_on_power_on_no_pullup.hex:a -Ulfuse:w:0xE1:m -Uhfuse:w:0xDF:m -Uefuse:w:0xFE:m
pause