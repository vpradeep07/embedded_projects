Designed to work with the "Sparkfun 7-Segment Serial Display"

https://www.sparkfun.com/products/11441

### Programming
Assuming you have the Atmel ICE

```
avr-g++ -mmcu=atmega328p -Wall -Os -o countdown.elf countdown.cpp
avr-objcopy -j .text -j .data -O ihex countdown.elf countdown.hex
sudo avrdude -p atmega328p -c atmelice_isp -U flash:w:countdown.hex
```
