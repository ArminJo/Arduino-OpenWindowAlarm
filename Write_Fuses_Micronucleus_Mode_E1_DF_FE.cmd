@echo off
color f0
title AvrDude Fuses Command Window
@echo. Writing ATtiny85 Lfuse to Micronucleus mode: E1 (16 MHz PLL, 1024 Clocks from Power Down sleep + 64 ms after reset), Hfuse to DF (BOD disable) and EFuse to FE (self programming enabled) 
@echo. trying to connect to device...
avrdude -p ATtiny85 -c stk500v1 -P COM6  -b 19200 -Ulfuse:w:0xE1:m -Uhfuse:w:0xDF:m -Uefuse:w:0xFE:m
pause