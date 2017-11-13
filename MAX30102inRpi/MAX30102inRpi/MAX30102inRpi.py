
from smbus2 import SMBus
import wiringpi as wiry

class Sense:
    STORAGE_SIZE = 4
    red = [0] * STORAGE_SIZE
    IR = [0] * STORAGE_SIZE
    Head = -1
    Tail = -1


class MAX30102:
    # INTERRUPT STATE REGISTERS
    MAX30102_INSTAT1 = 0x00
    MAX30102_INSTAT2 = 0x01
    MAX30102_INTENABLE1 = 0x02
    MAX30102_INTENABLE2 = 0x03

    # FIFO BUS REGISTERS
    MAX30102_FIFOWRITEPTR = 0x04
    MAX30102_FIFOOVERFLOW = 0x05
    MAX30102_FIFOREADPTR = 0x06
    MAX30102_FIFODATAREG = 0x07

    # CONFIGURATION REGISTERS
    MAX30102_FIFOCONFIG = 0x08
    MAX30102_MODECONFIG = 0x09
    MAX30102_SPO2CONFIG = 0x0A
    MAX30102_LED1_PULSEAMP = 0x0C
    MAX30102_LED2_PULSEAMP = 0x0D
    MAX30102_PROX_LED_PA = 0x10
    MAX30102_MULTILEDCONFIG1 = 0x11
    MAX30102_MULTILEDCONFIG2 = 0x12

    # DIE TEMPERATURE REGISTERS
    MAX30102_DIETEMPINT = 0x1F
    MAX30102_DIETEMPFRAC = 0X20
    MAX30102_DIETEMPCONFIG = 0X21

    # PROXIMITY FUNCTION REGISTERS
    MAX30102_PROXINTTHRES = 0X30

    # PARTID REGISTER
    MAX30102_REVISIONID = 0XFE
    MAX30102_PARTID = 0xFF

    # MAX30102 Commands
    MAX30102_INT_A_FULL_MASK = 0b10000000
    MAX30102_INT_A_FULL_ENABLE = 0x80
    MAX30102_INT_A_FULL_DISABLE = 0X00

    MAX30102_INT_A_DATA_RDY_MASK = 0b01000000
    MAX30102_INT_DATA_RDY_ENABLE = 0x40
    MAX30102_INT_DATA_RDY_DISABLE = 0x00

    MAX30102_INT_ALC_OVF_MASK = 0b00100000
    MAX30102_INT_ALC_OVF_ENABLE = 0x20
    MAX30102_INT_ALC_OVF_DISABLE = 0x00

    MAX30102_INT_PROX_INT_MASK = 0b00010000
    MAX30102_INT_PROX_INT_ENABLE = 0x10
    MAX30102_INT_PROX_INT_DISABLE = 0x00

    MAX30102_INT_DIE_TEMP_RDY_MASK = 0b00000010
    MAX30102_INT_DIE_TEMP_RDY_ENABLE = 0x02
    MAX30102_INT_DIE_TEMP_RDY_DISABLE = 0x00

    MAX30102_SAMPLEAVG_MASK = 0b11100000
    MAX30102_SAMPLEAVG_1 = 0x00
    MAX30102_SAMPLEAVG_2 = 0x20
    MAX30102_SAMPLEAVG_4 = 0x40
    MAX30102_SAMPLEAVG_8 = 0x60
    MAX30102_SAMPLEAVG_16 = 0x80
    MAX30102_SAMPLEAVG_32 = 0xA0

    MAX30102_ROLLOVER_MASK = 0xEF
    MAX30102_ROLLOVER_ENABLE = 0x10
    MAX30102_ROLLOVER_DISABLE = 0x00

    MAX30102_A_FULL_MASK = 0xF0

    # Mode configuration commands (page 19)
    MAX30102_SHUTDOWN_MASK = 0x7F
    MAX30102_SHUTDOWN = 0x80
    MAX30102_WAKEUP = 0x00

    MAX30102_RESET_MASK = 0xBF
    MAX30102_RESET = 0x40

    MAX30102_MODE_MASK = 0xF8
    MAX30102_MODE_REDONLY = 0x02  # HEARTRATE MODE (RED LED)
    MAX30102_MODE_IRONLY = 0x03  # SPO2 MODE (IR LED)
    MAX30102_MODE_MULTILED = 0x07  # MULTILED (RED & IR)

    # SPO2 configuration commands (pgs 19-20)
    MAX30102_ADCRANGE_MASK = 0x9F
    MAX30102_ADCRANGE_2048 = 0x00
    MAX30102_ADCRANGE_4096 = 0x20
    MAX30102_ADCRANGE_8192 = 0x40
    MAX30102_ADCRANGE_16384 = 0x60

    MAX30102_SAMPLERATE_MASK = 0xE3
    MAX30102_SAMPLERATE_50 = 0x00
    MAX30102_SAMPLERATE_100 = 0x04
    MAX30102_SAMPLERATE_200 = 0x08
    MAX30102_SAMPLERATE_400 = 0x0C
    MAX30102_SAMPLERATE_800 = 0x10
    MAX30102_SAMPLERATE_1000 = 0x14
    MAX30102_SAMPLERATE_1600 = 0x18
    MAX30102_SAMPLERATE_3200 = 0x1C

    MAX30102_PULSEWIDTH_MASK = 0xFC
    MAX30102_PULSEWIDTH_69 = 0x00
    MAX30102_PULSEWIDTH_118 = 0x01
    MAX30102_PULSEWIDTH_215 = 0x02
    MAX30102_PULSEWIDTH_411 = 0x03

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

    MAX30102_EXPECTED_PARTID = 0x15
    # REGISTER DEFINITION END
        
    BUS = 1
    max102 = SMBus(BUS) 
    MAX30102_ADDRESS = 0x57
    activeLeds = 3

    ##########################################################################
    #                                                                        #
    #                       Interrupt Configuration                          #
    ##########################################################################

    def readint1(self, max30102_address, max30102_instat1):
        max102 = self.max102

        inst1 = max102.read_byte_data(max30102_address, max30102_instat1)
        print("Interruption Status 1: ", inst1)
        return inst1

    def readint2(self, max30102_address, max30102_instat2):
        max102 = self.max102
        inst2 = max102.read_byte_data(max30102_address, max30102_instat2)
        print("Interruption Status 2: ", inst2)
        return inst2

    def bitmask(self, reg, mask, thing):
        max102 = self.max102
        originalContents = max102.read_byte_data(self.MAX30102_ADDRESS, reg)
        originalContentsM = originalContents & mask
        max102.write_byte_data(self.MAX30102_ADDRESS, reg, originalContentsM | thing)
        return True

    def enableAfull(self):
        intenable1 = self.MAX30102_INTENABLE1
        int_a_full_mask = self.MAX30102_A_FULL_MASK
        int_a_full_enable = self.MAX30102_INT_A_FULL_ENABLE

        self.bitmask(intenable1, int_a_full_mask, int_a_full_enable)
        return True

    def disableAfuill(self):
        intenable1 = self.MAX30102_INTENABLE1
        int_a_full_mask = self.MAX30102_A_FULL_MASK
        int_a_full_disable = self.MAX30102_INT_A_FULL_DISABLE

        self.bitmask(intenable1, int_a_full_mask, int_a_full_disable)
        return True

    def enableDataRdy(self):
        intenable1 = self.MAX30102_INTENABLE1
        int_data_rdy_mask = self.MAX30102_INT_A_DATA_RDY_MASK
        int_data_rdy_enable = self.MAX30102_INT_DATA_RDY_ENABLE
        self.bitmask(self, intenable1, int_data_rdy_mask, int_data_rdy_enable)
        return True

    def disableDataRdy(self):
        intenable1 = self.MAX30102_INTENABLE1
        int_data_rdy_mask = self.MAX30102_INT_A_DATA_RDY_MASK
        int_data_rdy_disable = self.MAX30102_INT_DATA_RDY_DISABLE
        self.bitmask(self, intenable1, int_data_rdy_mask, int_data_rdy_disable)
        return True

    def enabbleALCOVF(self):
        intenable1 = self.MAX30102_INTENABLE1
        int_alc_ovf_mask = self.MAX30102_INT_ALC_OVF_MASK
        int_alc_ovf_disable = self.MAX30102_INT_ALC_OVF_DISABLE
        self.bitmask(self, intenable1, int_alc_ovf_mask, int_alc_ovf_disable)
        return True

    def disableALCOVF(self):
        intenable1 = self.MAX30102_INTENABLE1
        int_alc_ovf_mask = self.MAX30102_INT_ALC_OVF_MASK
        int_alc_ovf_disable = self.MAX30102_INT_ALC_OVF_DISABLE
        self.bitmask(self, intenable1, int_alc_ovf_mask, int_alc_ovf_disable)
        return True

    def enablePROXINT(self):
        intenable1 = self.MAX30102_INTENABLE1
        int_prox_int_max = self.MAX30102_INT_PROX_INT_MASK
        int_prox_int_enable = self.MAX30102_INT_PROX_INT_ENABLE
        self.bitmask(self, intenable1, int_prox_int_max, int_prox_int_enable)
        return True

    def disablePROXINT(self):
        intenable1 = self.MAX30102_INTENABLE1
        int_prox_int_max = self.MAX30102_INT_PROX_INT_MASK
        int_prox_int_disable = self.MAX30102_INT_PROX_INT_DISABLE
        self.bitmask(self, intenable1, int_prox_int_max, int_prox_int_disable)
        return True

    def enableDIETEMPRDY(self):
        intenable2 = self.MAX30102_INTENABLE2
        int_die_temp_rdy_mask = self.MAX30102_INT_DIE_TEMP_RDY_MASK
        int_die_temp_rdy_enable = self.MAX30102_INT_DIE_TEMP_RDY_ENABLE

        self.bitmask(self, intenable2, int_die_temp_rdy_mask, int_die_temp_rdy_enable)
        return True

    def disbaleDIETEMPRDY(self):
        intenable2 = self.MAX30102_INTENABLE2
        int_die_temp_rdy_mask = self.MAX30102_INT_DIE_TEMP_RDY_MASK
        int_die_temp_rdy_disable = self.MAX30102_INT_DIE_TEMP_RDY_DISABLE
        self.bitmask(self, intenable2, int_die_temp_rdy_mask, int_die_temp_rdy_disable)
        return True

    def begintest(self, max30102_partid, max30102_expected_partid):
        max102 = self.max102
        address = self.MAX30102_ADDRESS
        partid = max102.read_byte_data(address, max30102_partid)

        if max30102_expected_partid == partid:
            print("Device Found")
        else:
            print("Error device not found")


    def softReset(self):
        max102 = self.max102
        wirp = wiry.millis()
        self.bitmask(self.MAX30102_MODECONFIG, self.MAX30102_RESET_MASK, self.MAX30102_RESET)
        #Poll for bit to clear, reset is then complete
        #time out after 100ms
        startTIme = wirp
        while wiry.millis()-startTIme < 100:
            response = max102.read_byte_data(self.MAX30102_ADDRESS, self.MAX30102_MODECONFIG)
            if (response & self.MAX30102_RESET) == 0:
                break
            wiry.delay(1)
        return True

    def shutdown(self):
        modeconfig = self.MAX30102_MODECONFIG
        shutdown_mask = self.MAX30102_SHUTDOWN_MASK
        shutdown = self.MAX30102_SHUTDOWN
        self.bitmask(modeconfig, shutdown_mask, shutdown)
        return True


    def wakeUp(self):
        modeconfig = self.MAX30102_MODECONFIG
        shutdown_mask = self.MAX30102_SHUTDOWN_MASK
        wakeup = self.MAX30102_WAKEUP
        self.bitmask(modeconfig, shutdown_mask, wakeup)
        return True


    def setLEDMode(self, mode):
        modeconfig = self.MAX30102_MODECONFIG
        mode_mask = self.MAX30102_MODE_MASK
        self.bitmask(modeconfig, mode_mask, mode)
        return True


    def setADCRange(self, adcRange):
        # adcRange: on of the MAX30102_ADCRANGE_2048, 4096, 8291, 16384
        spo2config = self.MAX30102_SPO2CONFIG
        adcrange_mask = self.MAX30102_ADCRANGE_MASK
        self.bitmask(spo2config, adcrange_mask, adcRange)
        return True


    def setSampleRate(self, sampleRate):
        #  sampleRate: one of the MAX30102_SAMPLERATE 50, 100, 200, 400, 800, 1000
        spo2config = self.MAX30102_SPO2CONFIG
        samplerate_mask = self.MAX30102_SAMPLERATE_MASK
        self.bitmask(spo2config, samplerate_mask, sampleRate)
        return True


    def setPulseWidth(self, pulseWidth):
        spo2config = self.MAX30102_SPO2CONFIG
        pulseWidth_mask = self.MAX30102_PULSEWIDTH_MASK
        self.bitmask(spo2config, pulseWidth_mask, pulseWidth)
        return True


    def setPulseAmplitudeRED(self, amplitude):
        max102 = self.max102
        led1_PA = self.MAX30102_LED1_PULSEAMP
        max102.write_byte_data(self.MAX30102_ADDRESS, led1_PA, amplitude)
        return True


    def setPulseAmplitudeIR(self, amplitude):
        max102 = self.max102
        led2_PA = self.MAX30102_LED2_PULSEAMP
        max102.write_byte_data(self.MAX30102_ADDRESS, led2_PA, amplitude)
        return True

    def setPulseAmplitudeProximity(self, amplitude):
        max102 = self.max102
        led_prox_amp = self.MAX30102_PROX_LED_PA
        max102.write_byte_data(self.MAX30102_ADDRESS, led_prox_amp, amplitude)
        return True

    def setProximityThreshold(self, threshMSB):
        max102 = self.max102
        proxintthresh = self.MAX30102_PROXINTTHRES
        max102.write_byte_data(self.MAX30102_ADDRESS, proxintthresh, threshMSB)
        return True

    def enableSlot(self, slotNumber, device):
        multiledconfig1 = self.MAX30102_MULTILEDCONFIG1
        multiledconfig2 = self.MAX30102_MULTILEDCONFIG2
        slot1_mask = self.MAX30102_SLOT1_MASK
        slot2_mask = self.MAX30102_SLOT2_MASK
        slot3_mask = self.MAX30102_SLOT3_MASK
        slot4_mask = self.MAX30102_SLOT4_MASK
        if slotNumber == 1:
            self.bitmask(multiledconfig1, slot1_mask, device)
        elif slotNumber == 2:
            self.bitmask(multiledconfig1, slot2_mask, device << 4)
        elif slotNumber == 3:
            self.bitmask(multiledconfig2, slot3_mask, device)
        elif slotNumber == 4:
            self.bitmask(multiledconfig2, slot4_mask, device << 4)

        return True

    def disableSlots(self):
        multiledconfig1 = self.MAX30102_MULTILEDCONFIG1
        multiledconfig2 = self.MAX30102_MULTILEDCONFIG2
        max102 = self.max102
        max102.write_byte_data(self.MAX30102_ADDRESS, multiledconfig1, 0)
        max102.write_byte_data(self.MAX30102_ADDRESS, multiledconfig2, 0)
        return True

    def setFIFOAverage(self, numberofsamples):
        fifoconfig = self.MAX30102_FIFOCONFIG
        sampleavg_mask = self.MAX30102_SAMPLERATE_MASK
        self.bitmask(fifoconfig, sampleavg_mask, numberofsamples)
        return True


    def clearFIFO(self):
        fifowriteptr = self.MAX30102_FIFOREADPTR
        fifooverflow = self.MAX30102_FIFOOVERFLOW
        fiforeadptr = self.MAX30102_FIFOREADPTR
        max102 = self.max102
        max102.write_byte_data(self.MAX30102_ADDRESS, fifowriteptr, 0)
        max102.write_byte_data(self.MAX30102_ADDRESS, fifooverflow, 0)
        max102.write_byte_data(self.MAX30102_ADDRESS, fiforeadptr, 0)
        return True


    def enableFIFORollover(self):
        self.bitmask(self.MAX30102_FIFOCONFIG, self.MAX30102_ROLLOVER_MASK, self.MAX30102_ROLLOVER_ENABLE)
        return True

    def disableFIFORollover(self):
        self.bitmask(self.MAX30102_FIFOCONFIG, self.MAX30102_ROLLOVER_MASK, self.MAX30102_ROLLOVER_DISABLE)
        return True

    # Set number of samples to trigger the almost full interrupt(Page 18)
    # Power on default is 32 samples
    # Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples

    def setFIFOalmostFull(self, numberofSamples):
        self.bitmask(self.MAX30102_FIFOCONFIG, self.MAX30102_A_FULL_MASK, numberofSamples)
        return True


    # Read FIFO Writer Pointer
    def getWriterPointer(self):
        max102 = self.max102
        writerPointer = max102.read_byte_data(self.MAX30102_ADDRESS, self.MAX30102_FIFOWRITEPTR)
        return writerPointer

    def getReadPointer(self):
        max102 = self.max102
        readPointer = max102.read_byte_data(self.MAX30102_ADDRESS, self.MAX30102_FIFOREADPTR)
        return readPointer

    def readTemperature(self):
        max102 = self.max102
        # Step 1: Config die temperature register to take 1 temperature sample
        max102.write_byte_data(self, self.MAX30102_ADDRESS, self.MAX30102_DIETEMPCONFIG, 0x01)

        # Poll for bit to clear, reading is then complete
        # Timeout after 100 ms
        startTime = wiry.millis()
        while wiry.millis() - startTime < 100:
            response = max102.read_byte_data(self.MAX30102_ADDRESS, self.MAX30102_DIETEMPCONFIG)
            if (response & 0x01) == 0:
                break
            wiry.delay(1)

        # if (millis() - startTime >= 100) return (-999)
        # Step 2: Read die temperature register (integer)
        tempInt = max102.read_byte_data(self, self.MAX30102_ADDRESS, self.MAX30102_DIETEMPCONFIG)
        tempFrac = max102.read_byte_data(self, self.MAX30102_ADDRESS, self.MAX30102_DIETEMPFRAC)

        # Step 3: Calculate temperature (datasheet pg. 22)
        return tempInt + tempFrac * 0.0625

    def setProxIntThresh(self, val):
        max102 = self.max102
        max102.write_byte_data(self.MAX30102_ADDRESS, self.MAX30102_PROXINTTHRES, val)
        return True


    def readPartID(self):
        max102 = self.max102
        max102.read_byte_data(self.MAX30102_ADDRESS, self.MAX30102_PARTID)
        return True


    def readRevisionID(self):
        max102 = self.max102
        max102.read_byte_data(self.MAX30102_ADDRESS, self.MAX30102_REVISIONID)
        return True


    # Setup the sensor
    # The MAX30102 has many settings. By default we select:
    # Sample Average = 4
    # Mode = MultiLED
    # ADC Range = 16384 (62.5pA per LSB)
    # Sample Rate = 50
    # USe the default setup if you are just getting  started with the MAX30102 sensor

    def setup(self, powerlevel, sampleAvegrage, ledMode, sampleRate, pulseWidth, adcRange):

        if self.softReset():
            print "reset .... :)"

            # FIFO Configuration
            # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=
            # The chip will average multiple samples of the same type together if you wish
        if sampleAvegrage == 1:
            self.setFIFOAverage(self.MAX30102_SAMPLEAVG_1)  # No average per FIFO record
        elif sampleAvegrage == 2:
            self.setFIFOAverage(self.MAX30102_SAMPLEAVG_2)
        elif sampleAvegrage == 4:
            self.setFIFOAverage(self.MAX30102_SAMPLEAVG_4)
        elif sampleAvegrage == 8:
            self.setFIFOAverage(self.MAX30102_SAMPLEAVG_8)
        elif sampleAvegrage == 16:
            self.setFIFOAverage(self.MAX30102_SAMPLEAVG_16)
        elif sampleAvegrage == 32:
            self.setFIFOAverage(self.MAX30102_SAMPLEAVG_32)
        else:
            self.setFIFOAverage(self.MAX30102_SAMPLEAVG_4)

            # setFIFOAlmostFull(2), Set to 30 samples to trigger an 'Almost Full' interrupt
        self.enableFIFORollover()

        # =-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        # Mode Configuration
        # =-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

        if ledMode == 3:
            self.setLEDMode(self.MAX30102_MODE_MULTILED)  # watch all LED channels
            self.activeLeds = 2
        elif ledMode == 2:
            self.setLEDMode(self.MAX30102_MODE_IRONLY)  # IR + HR SPO2 mode
            self.activeLeds = 2
        else:
            self.setLEDMode(self.MAX30102_MODE_REDONLY)  # red only HR mode
            self.activeLeds = 1



            # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=
            # SpO2 Configuration
            # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=
        if adcRange < 4096:
            self.setADCRange(self.MAX30102_ADCRANGE_2048)  # 7.81pA per LSB
        elif adcRange < 8192:
            self.setADCRange(self.MAX30102_ADCRANGE_4096)  # 15.63pA per LSB
        elif adcRange < 16384:
            self.setADCRange(self.MAX30102_ADCRANGE_8192)  # 31.25pA per LSB
        elif adcRange == 16384:
            self.setADCRange(self.MAX30102_ADCRANGE_16384)  # 62.5pA per LSB
        else:
            self.setADCRange(self.MAX30102_ADCRANGE_2048)

        if sampleRate < 100:
            self.setSampleRate(self.MAX30102_SAMPLERATE_50)  # takes 50 samples per second
        elif sampleRate < 200:
            self.setSampleRate(self.MAX30102_SAMPLERATE_100)
        elif sampleRate < 400:
            self.setSampleRate(self.MAX30102_SAMPLERATE_200)
        elif sampleRate < 800:
            self.setSampleRate(self.MAX30102_SAMPLERATE_400)
        elif sampleRate < 1000:
            self.setSampleRate(self.MAX30102_SAMPLERATE_800)
        elif sampleRate < 1600:
            self.setSampleRate(self.MAX30102_SAMPLERATE_1000)
        elif sampleRate < 3200:
            self.setSampleRate(self.MAX30102_SAMPLERATE_1600)
        else:
            self.setSampleRate(self.MAX30102_SAMPLERATE_50)

        # The longer the pulse width the longer range of detection you'll have
        # At 69us and 0.4mA it's about 2 inches
        # At 411us and 0.4mA it's about 6 inches

        if pulseWidth < 118:
            self.setPulseWidth(self.MAX30102_PULSEWIDTH_69)
        elif pulseWidth < 215:
            self.setPulseWidth(self.MAX30102_PULSEWIDTH_118)
        elif pulseWidth < 411:
            self.setPulseWidth(self.MAX30102_PULSEWIDTH_215)
        elif pulseWidth == 411:
            self.setPulseWidth(self.MAX30102_PULSEWIDTH_411)
        else:
            self.setPulseWidth(self.MAX30102_PULSEWIDTH_69)

            # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=
            # LED Pulse Amplitude Configuration
            # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=
            # Default is 0x1F which gets us 6.4mA
            # powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
            # powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
            # powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
            # powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch
        self.setPulseAmplitudeRED(powerlevel)
        self.setPulseAmplitudeIR(powerlevel)
        self.setPulseAmplitudeProximity(powerlevel)

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=
        # Multi-LED Mode Configuration, Enable the reading of the three LEDs
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=
        self.enableSlot(1, self.SLOT_RED_LED)
        if ledMode > 1:
            self.enableSlot(2, self.SLOT_IR_LED)
            # enableSlot(1, SLOT_RED_PILOT);
            # enableSlot(2, SLOT_IR_PILOT);

        if self.clearFIFO():
            print "FIFO cleared"


    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=
    # Data Collection
    # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=
    # Transform to hex for concatenation
    def to_hex(self, d):
        return hex(d).split('x')[-1]

    def concatbyte(self, tempArray):
        b0 = tempArray[0]
        b1 = tempArray[1]
        b2 = tempArray[2]
        b3 = tempArray[3]
        res = int(self.to_hex(b3) + self.to_hex(b2) + self.to_hex(b1) + self.to_hex(b0), 16)
        return res

    # TELL CALLER HOW MANY SAMPLES ARE AVAILABLE

    def available(self): 
        numberOfSamples = Sense.Head - Sense.Tail
        if numberOfSamples < 0:
            numberOfSamples += Sense.STORAGE_SIZE
            return numberOfSamples, True


    def getRed(self):
        if self.safeCheck(250):
            redsample = Sense.red[Sense.Head]
            return redsample
        else:
            return 0

    def getIR(self):
        if self.safeCheck(250):
            IRhead = Sense.IR[Sense.Head]
            return IRhead
        else:
            return 0

    def getFIFORed(self):
        fiforedtail = Sense.red[Sense.Tail]
        return fiforedtail

    def getFIFOIR(self):
        fifoIRtail = Sense.IR[Sense.Tail]
        return fifoIRtail

    def readSample(self):
        return self.max102.read_i2c_block_data(self.MAX30102_ADDRESS,self.MAX30102_FIFODATAREG,6)

    def nextSample(self):
        max102 = self.max102
        if self.available():
            Sense.Tail = Sense.Tail + 1
            Sense.Tail = Sense.Tail % Sense.STORAGE_SIZE
            print "sense tail : ", Sense.Tail

    def lastCorrect (self, read):
        last = 15000
        if read >15000 & read <45000:
            last = read
        else:
            read = last
       
        return read

    def check(self):
        firsttime = 0
        readPointer =self.getReadPointer()
        writePointer = self.getWriterPointer()

        if readPointer == firsttime:
            readPointer += 1
            self.max102.write_byte_data(self.MAX30102_ADDRESS, self.MAX30102_FIFOREADPTR, readPointer)
        if readPointer != writePointer:   
           # print readPointer
            numberOfSamples = writePointer - readPointer
            if numberOfSamples < 0:
                numberOfSamples = numberOfSamples + 32
           # print numberOfSamples

            bytesLeftToRead = numberOfSamples * self.activeLeds * 3
            I2C_BUFFER_LENGTH = 32

            while bytesLeftToRead > 0:
                
                toGet = bytesLeftToRead
                if toGet > I2C_BUFFER_LENGTH:
                   toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (self.activeLeds * 3))
                bytesLeftToRead = bytesLeftToRead - toGet
                while toGet > 0:
                    
                    Sense.Head = Sense.Head + 1
                    Sense.Head = Sense.Head % Sense.STORAGE_SIZE       
                    
                    Samples = self.max102.read_i2c_block_data(self.MAX30102_ADDRESS, self.MAX30102_FIFODATAREG,self.activeLeds*3)
                    tempLongred = Samples[0] << 16 | Samples[1] << 8 | Samples[2]
                    tempLongred = tempLongred >> 2
                    
                    Sense.red[Sense.Head] = round((tempLongred*3.3)/262144,2)                          
                   
                    if self.activeLeds>1:
                        tempLongIR = Samples[3]<<16 |Samples[4] << 8 |Samples[5]
                        tempLongIR = tempLongIR >> 2
                      
                        Sense.IR[Sense.Head] = round((tempLongIR*3.3)/262144,2)
                     
                    toGet -= self.activeLeds * 3
                    
                    
                #else:
                    #print "Read and Write are the same"
                    #print "# Samples: ", numberOfSamples
            return numberOfSamples
    # Check for new data but give up after a certain amount of time
    # Returns true if new data was found
    # Returns false if new data was not found

    def safeCheck(self, maxTimetoCheck):
        markTime = wiry.millis()

        while 1:
            if wiry.millis() - markTime > maxTimetoCheck:
                return False
            if self.check():
                return True

            wiry.delay(1)



