@Call SetPath
@echo Upgrade Digispark Bootloader with micronucleus upload
launcher -cdigispark -Uflash:w:upgrade-t85_no_pullup.hex:i
pause