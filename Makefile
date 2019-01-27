i2c: 
	avr-gcc -mmcu=atmega328 -Wall i2cTest.c twimaster.c nokia5110.c  -o rune.hex

disp:
	avr-gcc -mmcu=atmega328 -Wall dispTest.c nokia5110.c  -o rune.hex
	
isr:
	avr-gcc -mmcu=atmega328 -Wall isrTest.c nokia5110.c  -o rune.hex

acc:
	avr-gcc -mmcu=atmega328 -Wall accAngle.c twimaster.c nokia5110.c  -o rune.hex

conn:
	sudo avrdude -B 5 -c usbasp -p m328

flash:
	sudo avrdude -B 5 -c usbasp -p m328 -U flash:w:rune.hex

fuse:
	sudo avrdude -B 5 -c usbasp -p m328 -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

clean:
	rm rune.hex
