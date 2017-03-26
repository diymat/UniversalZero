import spidev
import curses
from UniversalZero import UZ
import string
import sys
import time

registe_names=["NOP",
               "DAC READBACK",
               "ADC SEQUENCE",
               "GENERAL CFG",
               "ADC pin CFG",
               "DAC pin CFG",
               "PD CFG",
               "LDAC CFG",
               "GPIO WRITE CFG",
               "GPIO WRITE DATA",
               "GPIO READ CFG",
               "PD & REF CFG",
               "OD CFG",
               "TS CFG",
               "RESERVED",
               "SFT RESET"
               ]




UZ0 = UZ(device = 1, port = 0, SPIspeed = 4000000)

#UZ0.spi_init(1,2,10000000,0b10)
UZ0.GPIO_Write_CR([0,2])
UZ0.GPIO_Read_CR([1])

#while True:
UZ0.EnableInternalVref()
UZ0.ADC_SetRange(1)
UZ0.DAC_SetRange(1)


try:
    UZ0.ADC_Pin_Cfg([7,5,3])
    UZ0.DAC_Pin_Cfg([6,4])
    iteration = 0
    screen = curses.initscr()
    screen.refresh()
    curses.start_color()
    screen.clear()
    scr = curses.newwin(23,80)

    curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLUE)
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED, curses.COLOR_BLACK)
	

    portl = 0
    state = 0

    FlashDelay = 5

    minfps = 999999
    maxfps = -9999999

    #cmilis = time.
    time1 = time.time()
    while True:
        try:
            ypos = 0
            time2 = 0
            iteration = (iteration + 1) % 40950
            UZ0.DAC_Write(6, iteration / 10)
            UZ0.DAC_Write(4, (40950 - iteration) / 10)
            result = UZ0.ADC_Seq_Cfg([7,5,3], False, True)
            ADC3 = result[1][3]
            ADC5 = result[1][5]
            ADC7 = result[1][7]
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('ADC Raw:', curses.color_pair(1) | curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('  CH0    CH1    CH2    CH3    CH4    CH5    CH6    CH7  TEMPERATURE', curses.color_pair(2) |  curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('0x{:03X}  0x{:03X}  0x{:03X}  0x{:03X}  0x{:03X}  0x{:03X}  0x{:03X}  0x{:03X}  0x{:03X}'.format(*result[0]), curses.color_pair(3) |  curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('  CH0    CH1    CH2    CH3    CH4    CH5    CH6    CH7  TEMPERATURE', curses.color_pair(2) |  curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('{:5.3f}V {:5.3f}V {:5.3f}V {:5.3f}V {:5.3f}V {:5.3f}V {:5.3f}V {:5.3f}V {:5.3f}Celc'.format(*result[1]), curses.color_pair(3) |  curses.A_BOLD)
            result = UZ0.DAC_Readback([0,1,2,3,4,5,6,7])
            DAC4 = result[1][4]
            DAC6 = result[1][6]
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('DAC Readback:', curses.color_pair(1) | curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('  CH0    CH1    CH2    CH3    CH4    CH5    CH6    CH7  ', curses.color_pair(2) |  curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('0x{:03X}  0x{:03X}  0x{:03X}  0x{:03X}  0x{:03X}  0x{:03X}  0x{:03X}  0x{:03X}'.format(*result[0]), curses.color_pair(4) | curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('  CH0    CH1    CH2    CH3    CH4    CH5    CH6    CH7', curses.color_pair(2) |  curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('{:5.3f}V {:5.3f}V {:5.3f}V {:5.3f}V {:5.3f}V {:5.3f}V {:5.3f}V {:5.3f}V'.format(*result[1]), curses.color_pair(4) |  curses.A_BOLD)
            if (portl & (1 << FlashDelay)) == (1 << FlashDelay):
                if state == 0:
                    UZ0.GPIO_Write([[2,0],[0,0]])
                    state = 1
                else:
                    UZ0.GPIO_Write([[2,1],[0,1]])
                    state = 0
                portl = 0
            portl += 1
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('GPIOS (IN)  {:1d} {:1d} {:1d} {:1d} {:1d} {:1d} {:1d} {:1d}' . format(*UZ0.GPIO_Read_CR([0,1,2,3,4,5,6,7],True)), curses.color_pair(2) |  curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('GPIOS (OUT) {:1d} {:1d} {:1d} {:1d} {:1d} {:1d} {:1d} {:1d}' . format(*[1 - state, 1 - state,0,0,0,0,0,0]), curses.color_pair(2) |  curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('AD85592 Test Configutation:',curses.color_pair(1) | curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('PIN0 - GPIO OUT    ', curses.color_pair(2))
            if state == 0 :
                scr.addstr('RED ', curses.color_pair(4) | curses.A_BOLD)
            else:
                scr.addstr('    ', curses.color_pair(2))
            scr.addstr('LED', curses.color_pair(2))
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('PIN1 - GPIO IN     ', curses.color_pair(2) | curses.A_BOLD)
            scr.addstr('connected to PIN2', curses.color_pair(3) | curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('PIN2 - GPIO OUT    ', curses.color_pair(2) | curses.A_BOLD)
            scr.addstr('connected to PIN1', curses.color_pair(3) | curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('PIN3 - ADC IN      ', curses.color_pair(2) | curses.A_BOLD)
            scr.addstr('connected to pot ', curses.color_pair(3) | curses.A_BOLD)
            scr.addstr('{:5.3f}V'.format(ADC3), curses.color_pair(4) | curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('PIN4 - DAC OUT ', curses.color_pair(2) | curses.A_BOLD)
            scr.addstr('{:5.3f}V '.format(DAC4), curses.color_pair(4) | curses.A_BOLD)
            scr.addstr('connected to PIN5 (ADC) ', curses.color_pair(3) | curses.A_BOLD)
            scr.addstr('{:5.3f}V '.format(ADC5), curses.color_pair(4) | curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('PIN6 - DAC OUT ', curses.color_pair(2) | curses.A_BOLD)
            scr.addstr('{:5.3f}V '.format(DAC6), curses.color_pair(4) | curses.A_BOLD)
            scr.addstr('connected to PIN7 (ADC) ', curses.color_pair(3) | curses.A_BOLD)
            scr.addstr('{:5.3f}V '.format(ADC7), curses.color_pair(4) | curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('PIN5 - ADC IN  ', curses.color_pair(2) | curses.A_BOLD)
            scr.addstr('{:5.3f}V '.format(ADC5), curses.color_pair(4) | curses.A_BOLD)
            scr.addstr('connected to PIN4 (DAC) ', curses.color_pair(3) | curses.A_BOLD)
            scr.addstr('{:5.3f}V '.format(DAC4), curses.color_pair(4) | curses.A_BOLD)
            scr.move(ypos, 0)
            ypos += 1
            scr.addstr('PIN7 - ADC IN  ', curses.color_pair(2) | curses.A_BOLD)
            scr.addstr('{:5.3f}V '.format(ADC7), curses.color_pair(4) | curses.A_BOLD)
            scr.addstr('connected to PIN5 (DAC) ', curses.color_pair(3) | curses.A_BOLD)
            scr.addstr('{:5.3f}V '.format(DAC6), curses.color_pair(4) | curses.A_BOLD)
            scr.refresh()
            scr.move(ypos, 0)
            ypos += 1
            time2 = time.time()
            fps = 1/ (time2-time1);
            if minfps > fps:
                minfps = fps
            if maxfps < fps:
                maxfps = fps
            #scr.addstr('FPS:{:06.2f} MAX FPS:{:06.2f} MIN FPS:{:06.2f}'.format(fps,maxfps,minfps), curses.color_pair(2) | curses.A_BOLD)
            time1 = time.time()
        except curses.error:
            curses.endwin()
            print("Error: Terminal window is too small\r");
            quit()
except KeyboardInterrupt:
    UZ0.spi_close()
    curses.endwin()
