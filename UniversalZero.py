import spidev

class UZ:
    NOP = 0
    DAC_READ_BACK = 0b00001000
    ADC_SEQ_REGISTER = 0b00010000
    DAC_ADC_CR = 0b00011000
    ADC_PIN_CFG = 0b00100000
    DAC_PIN_CFG = 0b00101000
    PULL_DOWN_CFG = 0b00110000
    READBACK_LDAC = 0b00111000
    GPIO_WRITE_CFG = 0b01000000
    GPIO_WRITE_DATA = 0b01001000
    GPIO_READ_CFG = 0b01010000
    PWR_REF_CFG = 0b01011000
    GPIO_OD_CFG = 0b01100000
    THREE_STATE_CFG = 0b01101000
    RES = 0b01110000
    SOFT_RESETHI = 0b01111101
    SOFT_RESETLO = 0b10101100
    DAC_WRITE = 0b10000000

    def __init__(self, device = 0, port = 0, SPIspeed = 500000, SPImode = 0b10):
        self.spidev = device
        self.spiport = port
        self.speed = SPIspeed
        self.mode = SPImode
        self.spi = spidev.SpiDev()
        self.ConversionResults = [[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]
        self.Vref = 2.5
        self.Vrefi = 2.5
        self.DACVref = 2.5
        self.ADCref = 2.5
        self.ADC_DAC_ControlRegister = [UZ.DAC_ADC_CR, 0]
        self.PowerDown_Reference_ControlRegister = [UZ.PWR_REF_CFG, 0]
        self.Gain = 1
        self.LDAC_ModeRegister = [UZ.READBACK_LDAC, 0]
        self.spi.open(device, port)
        self.spi.mode = SPImode
        self.spi.max_speed_hz = SPIspeed

        #self.device_init();



    #def device_init(self):
    #    print(self.spi.mode, self.spi.max_speed_hz)
    #    exit()

    def ADC_SetRange(self, gain):
        if gain == 1:
            self.ADC_DAC_ControlRegister[1] &= ~(1 << 5)
            self.ADC_Set_Vref(self.Vrefi)
            self.Gain = 1
        if gain == 2:
            self.ADC_DAC_ControlRegister[1] |= (1 << 5)
            self. ADC_Set_Vref(self.Vrefi * 2)
            self.Gain = 2
        self.spi.xfer(self.ADC_DAC_ControlRegister)
    
        
    def DAC_SetRange(self, gain):
        if gain == 1:
            self.ADC_DAC_ControlRegister[1] &= ~(1 << 4)
            self.DAC_Set_Vref(self.Vrefi)
        if gain == 2:
            self.ADC_DAC_ControlRegister[1] |= (1 << 4)
            self.DAC_Set_Vref(self.Vrefi * 2)
        self.spi.xfer(self.ADC_DAC_ControlRegister)


    def ADC_BufferPrecharge(self, precharge):
        if precharge == 0:
            self.ADC_DAC_ControlRegister[0] &= ~(precharge << 1)
        if precharge == 1:
            self.ADC_DAC_ControlRegister[0] |= (precharge << 1)
        self.spi.xfer(self.ADC_DAC_ControlRegister)


    def ADC_BufferEnable(self, enable):
        if enable == 0:
            self.ADC_DAC_ControlRegister[0] &= ~(enable << 0)
        if enable == 1:
            self.ADC_DAC_ControlRegister[0] |= (enable << 0)
        self.spi.xfer(self.ADC_DAC_ControlRegister)


    def PinLock(self, lock):
        if lock == 0:
            self.ADC_DAC_ControlRegister[1] &= ~(lock << 7)
        if lock == 1:
            self.ADC_DAC_ControlRegister[1] |= (lock << 7)
        self.spi.xfer(self.ADC_DAC_ControlRegister)


    def AllDACs(self, allDACs):
        if allDACs == 0:
            self.ADC_DAC_ControlRegister[1] &= ~(allDACs << 6)
        if allDACs == 1:
            self.ADC_DAC_ControlRegister[1] |= (allDACs << 6)
        self.spi.xfer(self.ADC_DAC_ControlRegister)


    def EnableInternalVref(self):
        self.PowerDown_Reference_ControlRegister[0] |= (1 << 1)
        self.spi.xfer([self.PowerDown_Reference_ControlRegister[0],self.PowerDown_Reference_ControlRegister[1]])


    def DisableInternalVref(self):
        self.PowerDown_Reference_ControlRegister[0] &= ~(1 << 1)
        self.spi.xfer(self.PowerDown_Reference_ControlRegister)


    def spi_close(self):
        self.spi.close();


    def Reset(self):
        byteLo = UZ.SOFT_RESETLO
        byteHi = UZ.SOFT_RESETHI
        self.spi.xfer([byteHi,byteLo])


    def GPIO_Write_CR(self, gpios, busy = False):
        byteLo = 0
        byteHi = UZ.GPIO_WRITE_CFG

        for port in gpios:
            byteLo = byteLo | (1 << port)
        if busy:
            byteHi = byteHi + 1
        self.spi.xfer([byteHi,byteLo])
        return [byteHi, byteLo]


    def GPIO_ThreeState_CFG(self, gpios):
        byteLo = 0
        byteHi = UZ.THREE_STATE_CFG

        for port in gpios:
            byteLo = byteLo | (1 << port)
        self.spi.xfer([byteHi,byteLo])
        return [byteHi, byteLo]


    def GPIO_PullDown_CFG(self, gpios):
        byteLo = 0
        byteHi = UZ.GPIO_OD_CFG

        for port in gpios:
            byteLo = byteLo | (1 << port)
        self.spi.xfer([byteHi,byteLo])
        return [byteHi, byteLo]


    def PIN_Power_CFG(self, gpios, PD_ALL = 0, EN_REF = 0):
        byteLo = 0
        byteHi = UZ.PWR_REF_CFG | (PD_ALL << 2) | (EN_REF << 1)

        for port in gpios:
            byteLo = byteLo | (1 << port)
        self.spi.xfer([byteHi,byteLo])
        return [byteHi, byteLo]


    def GPIO_Read_CR(self, gpios, readback = False):
        resultGPIO = [0,0,0,0,0,0,0,0]
        byteLo = 0
        byteHi = UZ.GPIO_READ_CFG

        for port in gpios:
            byteLo = byteLo | (1 << port)
        if readback:
            byteHi = byteHi | (1 << 2)
        self.spi.xfer([byteHi, byteLo])
        if readback:
            data = self.spi.readbytes(2)
            for port in gpios:
                resultGPIO[port] = ((((data[0] & 0x0f) << 8) + data[1]) & (1 << port)) >> port
            return resultGPIO


    def GPIO_Write(self, gpios):   #[[pin,value][pin,value]...]
        byteLo = 0
        byteHi = UZ.GPIO_WRITE_DATA

        for port in gpios:
            if port[1] == 1:
                byteLo = byteLo | (1 << port[0])
        self.spi.xfer([byteHi, byteLo])
        return [byteHi, byteLo]


    def DAC_Pin_Cfg(self, gpios):
        byteLo = 0
        byteHi = UZ.DAC_PIN_CFG

        for port in gpios:
            byteLo = byteLo | (1 << port)
        self.spi.xfer([byteHi, byteLo])
        return [byteHi, byteLo]


    def DAC_Write(self,port,value):
        byteLo = int(value) & 0xff
        byteHi = UZ.DAC_WRITE | ((int(port) << 4) | (int(value) & 0xf00) >> 8)
        self.spi.xfer([byteHi, byteLo])
        return [byteHi, byteLo]


    def ADC_Pin_Cfg(self, gpios):
        byteLo = 0
        byteHi = UZ.ADC_PIN_CFG

        for port in gpios:
            byteLo = byteLo | (1 << port)
        self.spi.xfer([byteHi, byteLo])
        return [byteHi, byteLo]


    def ADC_Seq_Cfg(self, gpios, REPEAT = False, TEMP = False):
        ValidConversions = 0
        byteLo = 0
        byteHi = UZ.ADC_SEQ_REGISTER
        NumOfConversions = 0

        for port in gpios:
            byteLo = byteLo | (1 << port)
            NumOfConversions += 1

        if REPEAT:
            byteHi = byteHi | (1 << 1)
        if TEMP:
            byteHi = byteHi | (1 << 0)
            gpios.append(0b1000)
        self.spi.xfer([byteHi, byteLo])
        for converison in range(0, NumOfConversions + 2):
            data = self.spi.readbytes(2)
            channel = ((data[0] & 0xf0) >> 4)
            if channel not in gpios: continue
            ValidConversions += 1
            self.ConversionResults[0][channel] = ((data[0] & 0x0f) << 8) + data[1]
            if ValidConversions == len(gpios): break
        for port in gpios:
            if port == 8: #temperature sensor
                self.ConversionResults[1][port] = 25.0 + (self.ConversionResults[0][port] - 820) / 2654 # The formula from DS is rubbish Vref * ConversionResults[0][port] / 4095
            else:
                self.ConversionResults[1][port] = self.Vref * self.ConversionResults[0][port] / 4095
        return self.ConversionResults


    def DAC_Readback(self, gpios):
        byteLo = 0
        byteHi = UZ.DAC_READ_BACK
        results = [[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]]

        for port in gpios:
            byteLo = 0b11000 | port
            self.spi.xfer([byteHi, byteLo])
            data = self.spi.readbytes(2)
            if ((data[0] & 0b01110000) >> 4) == port:
                results[0][port] = ((data[0] & 0x0f) << 8) + data[1]
                results[1][port] = self.DACVref * results[0][port] / 4095
        return results



    def ADC_Set_Vref(self, vref):
        self.Vref = vref
        return 1


    def Set_Reference_Vref(self, vref):
        self.Vrefi = vref
        return 1


    def DAC_Set_Vref(self, vref):
        self.DACVref = vref
        return 1

    DAC_TransferImmediately     = 0b00
    DAC_StoreData               = 0x01
    DAC_Update                  = 0x10

    def LDAC_Mode(self, mode):
        self.LDAC_ModeRegister[1] &= ~(0b11)
        self.LDAC_ModeRegister[1] |= (mode & 0b11)

        self.spi.xfer(self.LDAC_ModeRegister)
        data = self.spi.readbytes(2)

        return data


    def RegisterReadback(self, register):
        self.LDAC_ModeRegister[0] &= ~(0b00111100)
        self.LDAC_ModeRegister[0] |= ((register & 0b01111000) >> 1) | (1 << 6) #enable readback

        self.spi.xfer(self.LDAC_ModeRegister)
        data = self.spi.readbytes(2)

        self.LDAC_ModeRegister[0] &= ~(0b01111100)
        return data

