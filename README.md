# UniversalZero
Universal Zero 8 port GPIO/ADC/DAC phat for 40pin Raspberry Pi

UniversalZero Python library and demo program. Connect IO7(ADC) with IO6(DAC) and IO5(ADC) with IO4(DAC).
Another ports are configred as described in the UZdemo.py.

Enjoy :)

PS To use SPI1 interface you need to upgade Raspbian to verison 4.9 - __`sudo rpi-update`__ or use C userspace library (root rights required)

You need to enable SPI interface(s)

To enable __SPI0__ interface run __`sudo raspi-config`__ in the terminal window or run __`sudo nano /boot/config.txt`__ and add (or uncomment) line: __`dtparam=spi=on`__ 

To enable __SPI1__ interface run __`sudo nano /boot/config.txt`__ and add (or uncomment) line: __`dtoverlay=spi1-3cs`__ 

http://www.diymat.co.uk
