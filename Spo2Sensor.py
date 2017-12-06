from smbus2 import SMBus
import numpy as np

def _get_valid(d,value):
    try:
        return d[value]
    except KeyError:
        raise KeyError("Value %s not valid, use one of: %s" % (value,','.join([str(s) for s in d.keys()])))

def _twos_complement(val,bits):
    if (val &(1<<(bits-1)))!=0:
        val = val- (1<<bits)
        return val

class Spo2Sensor (object):
    
    #REGISTERS 
     # INTERRUPT STATE REGISTERS
    INSTAT1 = 0x00
    INSTAT2 = 0x01
    INTENABLE1 = 0x02
    INTENABLE2 = 0x03

    #INTERRUPT CONFIGURATION MASKS
    INT_A_FULL_MASK = 0b10000000
    INT_A_FULL_ENABLE = 0x80
    INT_A_FULL_DISABLE = 0X00

    INT_A_DATA_RDY_MASK = 0b01000000
    INT_DATA_RDY_ENABLE = 0x40
    INT_DATA_RDY_DISABLE = 0x00

    INT_ALC_OVF_MASK = 0b00100000
    INT_ALC_OVF_ENABLE = 0x20
    INT_ALC_OVF_DISABLE = 0x00

    INT_PROX_INT_MASK = 0b00010000
    INT_PROX_INT_ENABLE = 0x10
    INT_PROX_INT_DISABLE = 0x00

    INT_DIE_TEMP_RDY_MASK = 0b00000010
    INT_DIE_TEMP_RDY_ENABLE = 0x02
    INT_DIE_TEMP_RDY_DISABLE = 0x00

    #FIFO BUS REGISTERS
    FIFOWRITEPTR = 0x04
    FIFOOVERFLOW = 0x05
    FIFOREADPTR = 0x06
    FIFODATAREG = 0x07
 
    # CONFIGURATION REGISTERS
    FIFOCONFIG = 0x08
    MODECONFIG = 0x09
    SPO2CONFIG = 0x0A
    LED1_PA = 0x0C
    LED2_PA = 0x0D
    PROXLED_PA = 0x10
    MULTILEDCONFIG1 = 0x11
    MULTILEDCONFIG2 = 0x12

    #FIFO CONFIGURATION MASKS
    SAMPLEAVG_MASK = 0b11100000
    SAMPLEAVG = {
        1:0x00,
        2:0X20,
        4:0X40, 
        8:0X60, 
        16:0X80, 
        32:0XA0
        }
    

    ROLLOVER_MASK = 0xEF
    ROLLOVER_ENABLE = 0x10
    ROLLOVER_DISABLE = 0x00

    A_FULL_MASK = 0xF0

    #MODE CONFIGURATION MASKS
    SHUTDOWN_MASK = 0x7F
    SHUTDOWN = 0x80
    WAKEUP = 0x00

    RESET_MASK = 0xBF
    RESET = 0x40

    MODE_MASK = 0xF8
    MODE = {
        'RED': 0X02,
        'SPO2':0X03,
        'MULTI':0X07}
    
    #SPO2 CONFIGURATION MASKS AND DICTIONARIES
    #ADC RESOLUTION 69:15bits, 118:16bits, 215: 17bits, 411: 18 bits
    ADCRANGE_MASK = 0x9F
    SAMPLERATE_MASK = 0xE3
    PULSEWIDTH_MASK = 0xFC

    ADCRANGE = {
        2048:0x00, 
        4096:0x20, 
        8192:0x40, 
        16384:0x60
        }   
    PULSE_WIDTH = { 
        69:  0x00, 
        118: 0x01, 
        215: 0x02, 
        411: 0x03
        }
    SAMPLE_RATE = { 
        50:0x00, 100:0x04, 
        200:0x08, 
        400:0x0C, 
        800:0x10, 
        1000:0x14, 
        1600:0x18, 
        3200:0x1C
        }
    LED_CURRENT = { 
        0: 0x00, 
        .2: 0x01, 
        .4: 0x02, 
        3.1:0x0F, 
        6.4: 0x1F, 
        12.5: 0x3F, 
        25.4:0x7F, 
        50:0xFF
        }
    FIFOALMOSTFULL={
        32:0x00,
        31:0x01,
        30:0x02,
        29:0x03,
        28:0x04,
        27:0x05,
        26:0x06,
        25:0x07,
        24:0x08,
        23:0x09,
        22:0x0A,
        21:0x0B,
        20:0x0C,
        19:0x0D,
        18:0x0E,
        17:0x0F
    }
    # Multi-LED Mode configuration (pg 22)
    MAX30102_SLOT1_MASK = 0xF8
    MAX30102_SLOT2_MASK = 0x8F
    MAX30102_SLOT3_MASK = 0xF8
    MAX30102_SLOT4_MASK = 0x8F

    SLOT_NONE = 0x00
    SLOT_RED_LED = 0x01
    SLOT_IR_LED = 0x02
    SLOT_NONE_PILOT = 0x04
    SLOT_RED_PILOT = 0x05
    SLOT_IR_PILOT = 0x06
    SLOT_GREEN_PILOT = 0x07
    
    #DIE TEMPERATURE REGISTERS
    DIETEMPINT = 0x1F
    DIETEMPFRAC = 0X20
    DIETEMPCONFIG = 0X21

    # PROXIMITY FUNCTION REGISTERS
    PROXINTTHRES = 0X30

    # PARTID REGISTER
    REVISIONID = 0XFE
    PARTID = 0xFF
    ADDRESS = 0x57
    BUS = 1
    i2c = SMBus(BUS)
   
    
    def __init__(self, mode = 'SPO2', ledCurrent = 6.4, sampleAvg = 1, sampleRate = 100, pulseWidth = 411, ADCrange = 16384):
        self.setLEDMode(mode)
        self.setFIFOAverage(sampleAvg)
        self.setLEDCurrent(ledCurrent)
        self.setSampleRate(sampleRate)
        self.setPulseWidth(pulseWidth)
        self.setADCRange(ADCrange)

        self.max_buffer_len = 10000
        self.buffer_red = np.array([])
        self.buffer_ir = np.array([])

        self.newSample = False
        
    @property
    def red(self):
        return self.buffer_red[-1] if self.buffer_red else None

    @property
    def ir(self):
        return self.buffer_ir[-1] if self.buffer_ir else None    

    def bitmask(self, reg, mask, thing):
        originalContents= self.i2c.read_byte_data(self.ADDRESS, reg)
        originalContetnsMasked= originalContents & mask
        self.i2c.write_byte_data(self.ADDRESS, reg, originalContetnsMasked | thing)

    
    def readintstat1 (self):
        intst1 = self.i2c.read_byte_data(self.ADDRESS,self.INSTAT1)
        return intst1
    
    def readintstat2 (self):
        intst2 = self.i2c.read_byte_data(self.ADDRESS, self.INSTAT2)
        return intst2

    def enableAfull(self):
        self.bitmask(self.INTENABLE1, self.INT_A_FULL_MASK, self.INT_A_FULL_ENABLE)

    def disableAfull(self):
        self.bitmask(self.INTENABLE1, self.A_FULL_MASK, self.INT_A_FULL_DISABLE)

    def enablePPGRDY(self):
        self.bitmask(self.INTENABLE1, self.INT_A_DATA_RDY_MASK, self.INT_DATA_RDY_ENABLE)

    def disablePPGRDY(self):
         self.bitmask(self.INTENABLE1, self.INT_A_DATA_RDY_MASK, self.INT_DATA_RDY_DISABLE)
    
    def enableALCOVF(self):
        self.bitmask(self.INTENABLE1, self.INT_ALC_OVF_MASK, self.INT_ALC_OVF_ENABLE)
    
    def disableALCOVF(self):
        self.bitmask(self.INTENABLE1,self.INT_ALC_OVF_MASK,self.INT_ALC_OVF_DISABLE)

    def enablePROXINT(self):
        self.bitmask(self.INTENABLE1, self.INT_PROX_INT_MASK,self.INT_PROX_INT_ENABLE)

    def disablePROXINT(self):
        self.bitmask(self.INTENABLE1, self.INT_PROX_INT_MASK, self.INT_PROX_INT_DISABLE)
    
    def enableDIETEMPRDY(self):
        self.bitmask(self.INTENABLE2, self.INT_DIE_TEMP_RDY_MASK, self.INT_DIE_TEMP_RDY_ENABLE)
    
    def disableDIETEMPRDY(self):
        self.bitmask(self.INTENABLE2, self.INT_DIE_TEMP_RDY_MASK, self.INT_DIE_TEMP_RDY_DISABLE)

    def shutdown(self):
        self.bitmask(self.MODECONFIG, self.SHUTDOWN_MASK, self.SHUTDOWN)
    
    def WAKEUP(self):
        self.bitmask(self.MODECONFIG, self.SHUTDOWN_MASK, self.WAKEUP)
    
    def reset(self):
        self.bitmask(self.MODECONFIG, self.RESET_MASK, self.RESET)
    
    def setLEDMode(self, mode):
        mode = _get_valid(self.MODE, mode)
        self.bitmask(self.MODECONFIG, self.MODE_MASK, mode)
        print "mode set: ", mode
    
    def setADCRange(self, ADCrange):
        ADCrange = _get_valid(self.ADCRANGE, ADCrange)
        self.bitmask(self.SPO2CONFIG, self.ADCRANGE_MASK,ADCrange)
        print "ADC at ", ADCrange

    def setSampleRate(self, sampleRate):
        sampleRate = _get_valid(self.SAMPLE_RATE, sampleRate)
        self.bitmask(self.SPO2CONFIG, self.SAMPLERATE_MASK, sampleRate)

    def setPulseWidth(self, pulseWidth):
        pulseWidth = _get_valid(self.PULSE_WIDTH, pulseWidth)
        self.bitmask(self.SPO2CONFIG, self.PULSEWIDTH_MASK, pulseWidth)

    def setLEDCurrent(self,led_current = 6.4):
        led_current = _get_valid(self.LED_CURRENT,led_current)
        self.i2c.write_byte_data(self.ADDRESS,self.LED1_PA,led_current)
        self.i2c.write_byte_data(self.ADDRESS,self.LED2_PA, led_current)

    def setProxCurrent(self, proxCurrent = 6.4):
        proxCurrent = _get_valid(self.LED_CURRENT,proxCurrent)
        self.i2c.write_byte_data(self.ADDRESS, self.PROXLED_PA, proxCurrent)

    def setProxThreshold(self, threshold):
        self.i2c.write_byte_data(self.ADDRESS, self.PROXINTTHRES, threshold)

    def setFIFOAverage(self, numberofSamples):
        numberofSamples = _get_valid(self.SAMPLEAVG, numberofSamples)
        self.bitmask(self.FIFOCONFIG, self.SAMPLEAVG_MASK, numberofSamples)
    
    def clearFIFO(self):
        self.i2c.write_byte_data(self.ADDRESS, self.FIFOWRITEPTR, 0)
        self.i2c.write_byte_data(self.ADDRESS, self.FIFOREADPTR, 0)
        self.i2c.write_byte_data(self.ADDRESS, self.FIFOOVERFLOW, 0)

    def enableFIFOROllover(self):
        self.bitmask(self.FIFOCONFIG,self.ROLLOVER_MASK ,self.ROLLOVER_ENABLE)
    
    def diableFIFORollober(self):
        self.bitmask(self.FIFOCONFIG, self.ROLLOVER_MASK, self.ROLLOVER_DISABLE)

    # Set number of samples to trigger the almost full interrupt(Page 18)
    # Power on default is 32 samples
    # Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
        
    def setFIFOAF(self, numberofSamples):
        numberofSamples = _get_valid(self.FIFOALMOSTFULL, numberofSamples)
        self.bitmask(self.FIFOCONFIG, self.A_FULL_MASK, numberofSamples)

    def getWritePointer(self):
        writePointer = self.i2c.read_byte_data(self.ADDRESS, self.FIFOWRITEPTR)
        return writePointer

    def getReadPointer(self):
        readPointer = self.i2c.read_byte_data(self.ADDRESS, self.FIFOREADPTR)
        return readPointer
    
    def getTemperature(self):
        intg= _twos_complement(self.i2c.read_byte_data(self.ADDRESS, self.DIETEMPINT))
        frac = self.i2c.read_byte_data(self.ADDRESS, self.DIETEMPFRAC)
        return intg + (frac*0.0625)

    def getRevID(self):
        revID = self.i2c.read_byte_data(self.ADDRESS, self.REVISIONID)
        return revID

    def getPartID(self):
        partID = self.i2c.read_byte_data(self.ADDRESS, self.PARTID)
        return partID

    def getNumberofSamples(self):
        writePointer = self.getWritePointer()
        readPointer = self.getReadPointer()
        numberofSamples = abs((writePointer-readPointer)+32)%32
        if writePointer -readPointer == 0:
             numberofSamples = 32
        #print numberofSamples
        return numberofSamples

    def sampleAvailable(self):
       self.newSample = True
       

    def readSample(self):
        Samples = self.i2c.read_i2c_block_data(self.ADDRESS,self.FIFODATAREG,6)
       
        HR = 0
        IR = 0

        IR = (Samples[0]<<16) | (Samples[1]<<8) | Samples[2]
        IR = IR & 0x3FFFF
        HR = (Samples[3]<<16) | (Samples[4]<<8) | Samples[5]
        HR = HR & 0x3FFFF
        
        self.buffer_red = np.append(self.buffer_red,HR)
        self.buffer_ir = np.append(self.buffer_ir, IR)
        
        self.buffer_red = self.buffer_red[-self.max_buffer_len:]
        self.buffer_ir = self.buffer_ir[-self.max_buffer_len:]

    def ReadFIFOFULL (self):
        i=0
        self.clearFIFO()
        while i < 32:
            self.readSample()
            i+=1        

    def getBuffer(self, buffer):
       if buffer == "Red":
           return self.buffer_red
       elif buffer == "IR":
           return self.buffer_ir

    

    