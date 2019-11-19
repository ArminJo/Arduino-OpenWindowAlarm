@Call SetPath
REM The files t85_no_pullup.hex and t85_entry_on_power_on_no_pullup.hex are identical!
@echo Upgrade Digispark Bootloader with micronucleus upload
launcher -cdigispark -Uflash:w:upgrade-t85_entry_on_power_on_no_pullup.hex:i
pause