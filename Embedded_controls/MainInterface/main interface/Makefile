TARGET=main
MCU=atmega328p
SOURCES=lib/motorcontrol.c lib/stackmon.c lib/adc.c lib/gyro.c lib/i2c.c lib/ultrasonic.c lib/usart.c schnittstelle.c main_interface.c main.c lib/infrared.c  #$(shell find . -name "*.c")
PROGRAMMER=arduino
#auskommentieren für automatische Wahl
PORT=\\COM3 \

#Ab hier nichts verändern
OBJECTS=$(SOURCES:.c=.o)
CFLAGS=-c -Os -g -Os -Wall -mcall-prologues
LDFLAGS=-lm


unicorn: all program  connect #unicorn: all program #clean# connect

all: hex eeprom

hex: $(TARGET).hex

eeprom: $(TARGET)_eeprom.hex

$(TARGET).hex: $(TARGET).elf
	avr-objcopy -O ihex -j .data -j .text $(TARGET).elf $(TARGET).hex

$(TARGET)_eeprom.hex: $(TARGET).elf
	avr-objcopy -O ihex -j .eeprom --change-section-lma .eeprom=1 $(TARGET).elf $(TARGET)_eeprom.hex

$(TARGET).elf: $(OBJECTS)
	avr-gcc $(LDFLAGS) -mmcu=$(MCU) $(OBJECTS) -o $(TARGET).elf

.c.o:
	avr-gcc $(CFLAGS) -mmcu=$(MCU) $< -o $@

size:
	avr-size --	cu=$(MCU) -C $(TARGET).elf

program:
	avrdude -p$(MCU) -P$(PORT) -b57600  -c$(PROGRAMMER) -Uflash:w:$(TARGET).hex:a

#clean_tmp:
	#rm -rf *.o
	#rm -rf *.elf

#clean:
	#rm -rf *.o
	#rm -rf ./lib/*.o
	#rm -rf *.elf
	#rm -rf *.hex

connect:
	screen $(PORT) 9600
