avrdude.exe -p attiny85  -c usbtiny -U flash:w:tn85eforth.hex

@rem read the fuses
@rem avrdude.exe -p attiny85  -c usbtiny -v

@rem write the fuses - 1 MHz
@rem avrdude.exe -p attiny85  -c usbtiny -U lfuse:w:0x62:m

@rem write the fuses - 8 MHz - use this!
@rem avrdude.exe -p attiny85  -c usbtiny -U lfuse:w:0xE2:m

@rem write the fuses - enable SPM instruction
@rem avrdude.exe -p attiny85  -c usbtiny -U efuse:w:0xFE:m

@rem perform chip erase
@rem avrdude.exe -p attiny85  -c usbtiny -v -e
