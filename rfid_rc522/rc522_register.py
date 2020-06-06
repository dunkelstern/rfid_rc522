from typing import NewType

RC522Register = NewType('RC522Register', int)  # Register definition
Command       = NewType('Command', int)        # Command definitions
ModemState    = NewType('ModemState', int)     # Modem states
CRCPreset     = NewType('CRCPreset', int)      # CRC Presets
TrxSpeed      = NewType('TrxSpeed', int)       # Recv/Transmission speeds
TxDriver      = NewType('TxDriver', int)       # Selection of TX Driver
MFOut         = NewType('MFOut', int)          # MFOut pin selection
UARTSel       = NewType('UARTSel', int)        # RC UART mode select
SerialSpeed   = NewType('SerialSpeed', int)    # Serial speed for RS232 interface
RxGain        = NewType('RxGain', int)         # Receiver gain
GatedMode     = NewType('GatedMode', int)      # Timer Gated mode

#
# Register definitions
#

# Page 0: Command and status
CommandReg      = RC522Register(0x01)  # starts and stops command execution
ComlEnReg       = RC522Register(0x02)  # enable and disable interrupt request control bits
DivlEnReg       = RC522Register(0x03)  # enable and disable interrupt request control bits
ComIrqReg       = RC522Register(0x04)  # interrupt request bits
DivIrqReg       = RC522Register(0x05)  # interrupt request bits
ErrorReg        = RC522Register(0x06)  # error bits showing the error status of the last command executed
Status1Reg      = RC522Register(0x07)  # communication status bits
Status2Reg      = RC522Register(0x09)  # receiver and transmitter status bits
FIFODataReg     = RC522Register(0x09)  # input and output of 64 byte FIFO buffer
FIFOLevelReg    = RC522Register(0x0A)  # number of bytes stored in the FIFO buffer
WaterLevelReg   = RC522Register(0x0B)  # level for FIFO underflow and overflow warning
ControlReg      = RC522Register(0x0C)  # miscellaneous control registers
BitFramingReg   = RC522Register(0x0D)  # adjustments for bit-oriented frames
CollReg         = RC522Register(0x0E)  # bit position of the first bit-collision detected on the RF interface

# Page 1: Command
ModeReg         = RC522Register(0x11)  # defines general modes for transmitting and receiving
TxModeReg       = RC522Register(0x12)  # defines transmission data rate and framing
RxModeReg       = RC522Register(0x13)  # defines reception data rate and framing
TxControlReg    = RC522Register(0x14)  # controls the logical behavior of the antenna driver pins TX1 and TX2
TxASKReg        = RC522Register(0x15)  # controls the setting of the transmission modulation
TxSelReg        = RC522Register(0x16)  # selects the internal sources for the antenna driver
RxSelReg        = RC522Register(0x17)  # selects internal receiver settings
RxThresholdReg  = RC522Register(0x18)  # selects thresholds for the bit decoder
DemodReg        = RC522Register(0x19)  # defines demodulator settings
MfTxReg         = RC522Register(0x1C)  # controls some MIFARE communication transmit parameters
MfRxReg         = RC522Register(0x1D)  # controls some MIFARE communication receive parameters
SerialSpeedReg  = RC522Register(0x1F)  # selects the speed of the serial UART interface

# Page 2: Configuration
CRCResultRegMSB = RC522Register(0x21)  # shows the MSB value of the CRC calculation
CRCResultRegLSB = RC522Register(0x22)  # shows the LSB value of the CRC calculation
ModWidthReg     = RC522Register(0x24)  # controls the ModWidth setting
RFCfgReg        = RC522Register(0x26)  # configures the receiver gain
GsNReg          = RC522Register(0x27)  # selects the conductance of the antenna driver pins TX1 and TX2 for modulation
CWGsPReg        = RC522Register(0x28)  # defines the conductance of the p-driver output during periods of no modulation
ModGsPReg       = RC522Register(0x29)  # defines the conductance of the p-driver output during periods of modulation
TModeReg        = RC522Register(0x2A)  # defines settings for the internal timer
TPrescalerReg   = RC522Register(0x2B)  # "
TReloadRegHi    = RC522Register(0x2C)  # defines the 16-bit timer reload value
TReloadRegLo    = RC522Register(0x2D)  # "
TCounterValRegHi= RC522Register(0x2E)  # shows the 16-bit timer value
TCounterValRegLo= RC522Register(0x2E)  # "


# Page 3: Test register
TestSel1Reg     = RC522Register(0x31)  # general test signal configuration
TestSel2Reg     = RC522Register(0x32)  # general test signal configuration and PRBS control
TestPinEnReg    = RC522Register(0x33)  # enables pin output driver on pins D1 to D7
TestPinValueReg = RC522Register(0x34)  # defines the values for D1 to D7 when it is used as an I/O bus
TestBusReg      = RC522Register(0x35)  # shows the status of the internal test bus
AutoTestReg     = RC522Register(0x36)  # controls the digital self test
VersionReg      = RC522Register(0x37)  # shows the software version
AnalogTestReg   = RC522Register(0x38)  # controls the pins AUX1 and AUX2
TestDAC1Reg     = RC522Register(0x39)  # defines the test value for TestDAC1
TestDAC2Reg     = RC522Register(0x3A)  # defines the test value for TestDAC2
TestADCReg      = RC522Register(0x3B)  # shows the value of ADC I and Q channels


#
# Register Bit definitions
#

# CommandReg register: Starts and stops command execution

CommandReg_RcvOff    = (1 << 5)  # set: analog part of the receiver is switched off
CommandReg_PowerDown = (1 << 4)  # set: Soft power-down mode entered

# Command[3:0]
Command_Idle             = Command(0b0000)  # no action, cancels current command execution
Command_Mem              = Command(0b0001)  # stores 25 bytes into the internal buffer
Command_GenerateRandomID = Command(0b0010)  # generates a 10-byte random ID number
Command_CalcCRC          = Command(0b0011)  # activates the CRC coprocessor or performs a self test
Command_Transmit         = Command(0b0100)  # transmits data from the FIFO buffer
Command_NoCmdChange      = Command(0b0111)  # no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
Command_Receive          = Command(0b1000)  # activates the receiver circuits
Command_Transceive       = Command(0b1100)  # transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
Command_MFAuthent        = Command(0b1110)  # performs the MIFARE standard authentication as a reader
Command_SoftReset        = Command(0b1111)  # resets the MFRC522

def make_CommandReg(command: Command, receiver_enabled: bool=True, powered_down: bool=False) -> bytes:
    """
    Create a ``CommandReg`` register value from parameters

    :param Command command: The command to send
    :param bool receiver_enabled: Enable the Receiver/RF Field
    :param bool powered_down: Soft-Powerdown the module to save energy
    :returns: a bytes-object with one byte containing the register values
    """
    result = Command
    if not receiver_enabled:
        result |= CommandReg_RcvOff
    if powered_down:
        result |= CommandReg_PowerDown
    return bytes([result])


# ComIEnReg register: Control bits to enable and disable the passing of interrupt requests.

# set: signal on pin IRQ is inverted with respect to the Status1Reg register’s IRq bit
# unset: signal on pin IRQ is equal to the IRq bit; in combination with the DivIEnReg register’s IRqPushPull bit,
#        the default value of logic 1 ensures that the output level on pin IRQ is 3-state
ComIEnReg_IRqInv     = (1 << 7)
ComIEnReg_TxIEn      = (1 << 6)  # allows the transmitter interrupt request (TxIRq bit) to be propagated to pin IRQ
ComIEnReg_RxIEn      = (1 << 5)  # allows the receiver interrupt request (RxIRq bit) to be propagated to pin IRQ
ComIEnReg_IdleIEn    = (1 << 4)  # allows the idle interrupt request (IdleIRq bit) to be propagated to pin IRQ
ComIEnReg_HiAlertIEn = (1 << 3)  # allows the high alert interrupt request (HiAlertIRq bit) to be propagated to pin IRQ
ComIEnReg_LoAlertIEn = (1 << 2)  # allows the low alert interrupt request (LoAlertIRq bit) to be propagated to pin IRQ
ComIEnReg_ErrIEn     = (1 << 1)  # allows the error interrupt request (ErrIRq bit) to be propagated to pin IRQ
ComIEnReg_TimerIEn   = (1 << 0)  # allows the timer interrupt request (TimerIRq bit) to be propagated to pin IRQ


# DivIEnReg register: Control bits to enable and disable the passing of interrupt requests
DivIEnReg_IRQPushPull = (1 << 7)  # set: pin IRQ is a standard CMOS output pin, unset: pin IRQ is an open-drain output pin
DivIEnReg_MfinActIEn  = (1 << 4)  # allows the MFIN active interrupt request to be propagated to pin IRQ
DivIEnReg_CRCIEn      = (1 << 2)  # allows the CRC interrupt request, indicated by the DivIrqReg register’s CRCIRq bit, to be propagated to pin IRQ


# ComIrqReg register: Interrupt request bits

# set: indicates that the marked bits in the ComIrqReg register are set,
# unset: indicates that the marked bits in the ComIrqReg register are cleared
ComIrqReg_Set1       = (1 << 7)  

ComIrqReg_TxIRq      = (1 << 6)  # set immediately after the last bit of the transmitted data was sent out

# receiver has detected the end of a valid data stream if the RxModeReg register’s RxNoErr bit is set to logic 1,
# the RxIRq bit is only set to logic 1 when data bytes are available in the FIFO
ComIrqReg_RxIRq      = (1 << 5)

#  If a command terminates, for example, when the CommandReg changes its value from any command to the Idle
#  command if an unknown command is started, the CommandReg register Command[3:0] value changes to the idle
# state and the IdleIRq bit is set The microcontroller starting the Idle command does not set the IdleIRq bit
ComIrqReg_IdleIRq    = (1 << 4)

# the Status1Reg register’s HiAlert bit is set in opposition to the HiAlert bit, the HiAlertIRq bit stores
# this event and can only be reset as indicated by the Set1 bit in this register
ComIrqReg_HiAlertIRq = (1 << 3)

# Status1Reg register’s LoAlert bit is set in opposition to the LoAlert bit, the LoAlertIRq bit stores this
# event and can only be reset as indicated by the Set1 bit in this register
ComIrqReg_LoAlertIRq = (1 << 2)

ComIrqReg_ErrIRq     = (1 << 1)  # any error bit in the ErrorReg register is set
ComIrqReg_TimerIRq   = (1 << 0)  # the timer decrements the timer value in register TCounterValReg to zero


# DivIrqReg register: Interrupt request bits

# set: indicates that the marked bits in the DivIrqReg register are set
# unset: indicates that the marked bits in the DivIrqReg register are cleared
DivIrqReg_Set2       = (1 << 7)

DivIrqReg_MfinActIRq = (4 << 1)  # MFIN is active this interrupt is set when either a rising or falling signal edge is detected
DivIrqReg_CRCIRq     = (2 << 1)  # the CalcCRC command is active and all data is processed


# ErrorReg register: Error bit register showing the error status of the last command executed.

# data is written into the FIFO buffer by the host during the MFAuthent command or if data is written into
# the FIFO buffer by the host during the time between sending the last bit on the RF interface and receiving
# the last bit on the RF interface
ErrorReg_WrErr       = (1 << 7)

# internal temperature sensor detects overheating, in which case the antenna drivers are automatically switched off
ErrorReg_TempErr     = (1 << 6)

# the host or a MFRC522’s internal state machine (e.g. receiver) tries to write data to the FIFO buffer even
# though it is already full
ErrorReg_BufferOvfl  = (1 << 4)

# a bit-collision is detected cleared automatically at receiver start-up phase only valid during the bitwise
# anticollision at 106 kBd always set to logic 0 during communication protocols at 212 kBd, 424 kBd and 848 kBd
ErrorReg_CollErr     = (1 << 3)

# the RxModeReg register’s RxCRCEn bit is set and the CRC calculation fails, automatically cleared to logic 0
# during receiver start-up phase
ErrorReg_CRCErr      = (1 << 2)

# parity check failed, automatically cleared during receiver start-up phase only valid for
# ISO/IEC 14443 A/MIFARE communication at 106 kBd
ErrorReg_ParityErr   = (1 << 1)

# set to logic 1 if the SOF is incorrect, automatically cleared during receiver start-up phase
# bit is only valid for 106 kBd during the MFAuthent command, the ProtocolErr bit is set to logic 1 if the
# number of bytes received in one data stream is incorrect
ErrorReg_ProtocolErr = (1 << 0)


# Status1Reg register: Contains status bits of the CRC, interrupt and FIFO buffer

# the CRC result is zero, for data transmission and reception, the CRCOk bit is undefined: use the
# ErrorReg register’s CRCErr bit indicates the status of the CRC coprocessor, during calculation the
# value changes to logic 0, when the calculation is done correctly the value changes to logic 1
Status1Reg_CRCOk    = (1 << 6)

# the CRC calculation has finished, only valid for the CRC coprocessor calculation using the CalcCRC command
Status1Reg_CRCReady = (1 << 5)

# indicates if any interrupt source requests attention with respect to the setting of the interrupt
# enable bits: see the ComIEnReg and DivIEnReg registers
Status1Reg_IRq      = (1 << 4)

# MFRC522’s timer unit is running, i.e. the timer will decrement the TCounterValReg register with the
# next timer clock
# Remark: in gated mode, the TRunning bit is set to logic 1 when the timer is enabled by TModeReg
# register’s TGated[1:0] bits; this bit is not influenced by the gated signal
Status1Reg_TRunning = (1 << 3)

# the number of bytes stored in the FIFO buffer corresponds to equation:
# example:
# FIFO length = 60, WaterLevel = 4 -> HiAlert = 1
# FIFO length = 59, WaterLevel = 4 -> HiAlert = 0
Status1Reg_HiAlert  = (1 << 1)

# the number of bytes stored in the FIFO buffer corresponds to equation:
# example:
# FIFO length = 4, WaterLevel = 4 -> LoAlert = 1
# FIFO length = 5, WaterLevel = 4 -> LoAlert = 0
Status1Reg_LoAlert  = (1 << 0)


# Status2Reg register: Contains status bits of the receiver, transmitter and data mode detector

Status2Reg_TempSensClear = (1 << 7)  # clears the temperature error if the temperature is below the alarm limit of 125C
Status2Reg_I2CForceHS    = (1 << 6)  # set: the I2C-bus input filter is set to the High-speed mode independent of the I2C-bus protocol

# indicates that the MIFARE Crypto1 unit is switched on and therefore all data communication
# with the card is encrypted can only be set to logic 1 by a successful execution of the
# MFAuthent command only valid in Read/Write mode for MIFARE standard cards, this bit is cleared by software
Status2Reg_MFCrypto1On   = (1 << 3)

# ModemState[2:0]
ModemState_Idle          = ModemState(0b000)  # idle
ModemState_WaitStartSend = ModemState(0b001)  # wait for the BitFramingReg register’s StartSend bit
ModemState_TxWait        = ModemState(0b010)  # wait until RF field is present if the TModeReg register’s TxWaitRF bit is set to logic 1
ModemState_Transmitting  = ModemState(0b011)  # transmitting
ModemState_RxWait        = ModemState(0b100)  # wait until RF field is present if the TModeReg register’s TxWaitRF bit is set to logic 1
ModemState_WaitData      = ModemState(0b101)  # wait for data
ModemState_Receiving     = ModemState(0b110)  # receiving

def get_ModemState(status2Reg: bytes) -> ModemState:
    """
    Get modem state from register byte of ``Status2Reg`` register

    :param bytes status2reg: the value of ``Status2Reg`` register to decode
    :returns: decoded ``ModemState`` object
    """
    return ModemState(status2reg[0] & 0x03)


# FIFODataReg register: Input and output of 64 byte FIFO buffer

# FIFOData[7:0]: data input and output port for the internal 64-byte FIFO buffer, FIFO buffer acts
# as parallel in/parallel out converter for all serial data stream inputs and outputs


# FIFOLevelReg register: Indicates the number of bytes stored in the FIFO.

# immediately clears the internal FIFO buffer’s read and write pointer and ErrorReg register’s BufferOvfl bit
# reading this bit always returns 0
FIFOLevelReg_FlushBuffer = (1 << 7)

# FIFOLevel [6:0] indicates the number of bytes stored in the FIFO buffer, writing to the FIFODataReg
# register increments and reading decrements the FIFOLevel value

def get_FIFOLevel(fifoLevelReg: bytes) -> int:
    """
    Get fifo level from ``FIFOLevelReg`` register

    :param bytes fifoLevelReg: the value of the ``FIFOLevelReg`` register to decode
    :returns: fill state of the FIFO
    """
    return fifoLevelReg[0] & ~FIFOLevelReg_FlushBuffer


# WaterLevelReg register: Defines the level for FIFO under- and overflow warning.

# WaterLevel[5:0] Defines the level for FIFO under- and overflow warning.
# defines a warning level to indicate a FIFO buffer overflow or underflow:
# Status1Reg register’s HiAlert bit is set to logic 1 if the remaining number of bytes in the FIFO buffer
# space is equal to, or less than the defined number of WaterLevel bytes
# Status1Reg register’s LoAlert bit is set to logic 1 if equal to, or less than the WaterLevel bytes in
# the FIFO buffer

# HiAlert = (64 - FIFOLength) <= WaterLevel
# LoAlert = FIFOLength <= WaterLevel


# ControlReg register: Miscellaneous control bits

ControlReg_TStopNow  = (1 << 7)  # timer stops immediately, reading this bit always returns it to logic0
ControlReg_TStartNow = (1 << 6)  #timer starts immediately, reading this bit always returns it to logic 0

# RxLastBits[2:0]: indicates the number of valid bits in the last received byte if this value is 0b000,
#                  the whole byte is valid

def get_valid_bits(controlReg: bytes) -> int:
    return controlReg[0] & 0x07


# BitFramingReg register: Adjustments for bit-oriented frames

BitFramingReg_StartSend = (1 << 7)  #  starts the transmission of data, only valid in combination with the Transceive command
# [6:4] : RxAlign[2:0] used for reception of bit-oriented frames: defines the bit position for
#                      the first bit received to be stored in the FIFO buffer
# example:
# 0 -> LSB of the received bit is stored at bit position 0, the second received bit is stored at bit position 1
# 1 -> LSB of the received bit is stored at bit position 1, the second received bit is stored at bit position 2
# 7 -> LSB of the received bit is stored at bit position 7, the second received bit is stored in the next byte
#      that follows at bit position 0
#
# These bits are only to be used for bitwise anticollision at 106 kBd, for all other modes they are set to 0


# [2:0] : TxLastBits[2:0]
# used for transmission of bit oriented frames: defines the number of bits of the last byte that will be
# transmitted, 0b000 indicates that all bits of the last byte will be transmitted

def make_BitFramingReg(start_send: bool, alignment: int=0, skip_last: int=0) -> bytes:
    """
    Create a value to send to the ``BitFramingReg`` register

    :param bool start_send: Start transmission of data
    :param int alignment: used for reception of bit-oriented frames, defines the bit position for
                          the first bit received to be stored in the FIFO buffer
    :param int skip_last: used for transmission of bit oriented frames, defines the number of bits
                          of the last byte that will be transmitted
    :returns: single byte bytes object with register value
    """
    result = 0
    if start_send:
        result |= BitFramingReg_StartSend
    if alignment > 0b111:
        raise ValueError('`alignment` can have a max value of 0b111 but it has {}'.format(alignment))
    result |= (alignment << 4)
    if skip_last > 0b111:
        raise ValueError('`skip_last` can have a max value of 0b111 but it has {}'.format(skip_last))
    result |= skip_last
    return bytes([result])


# CollReg register: Defines the first bit-collision detected on the RF interface

CollReg_ValuesAfterColl = (1 << 7)  # all received bits will be cleared after a collision, only used during bitwise anticollision at 106 kBd, otherwise 1
CollReg_CollPosNotValid = (1 << 5)  # no collision detected or the position of the collision is out of the range of CollPos[4:0]

# [4:0] : CollPos[4:0] shows the bit position of the first detected collision in a received frame
#                      only data bits are interpreted
# example:
# 0x00 -> indicates a bit-collision in the 32nd bit
# 0x01 -> indicates a bit-collision in the 1st bit
# 0x08 -> indicates a bit-collision in the 8th bit
#
#  These bits will only be interpreted if the CollPosNotValid bit is set to logic 0

def get_CollPos(collReg: bytes) -> int:
    """
    Get first detected collision in received frame from ``CollReg`` register value

    :param bytes collReg: Register value
    :returns: Bit-position of first detected collision
    """
    col_pos = collReg & 0x1F
    if col_pos == 0:  # zero has a special meaning
        return 32
    return col_pos


# ModeReg register: Defines general mode settings for transmitting and receiving

ModeReg_MSBFirst = (1 << 7)  # CRC coprocessor calculates the CRC with MSB first
ModeReg_TxWaitRF = (1 << 5)  # transmitter can only be started if an RF field is generated
ModeReg_PolMFin  = (1 << 3)  # defines the polarity of pin MFIN, 1 -> Active HIGH, 0 -> Active LOW

# CRCPreset[1:0] defines the preset value for the CRC coprocessor for the CalcCRC command
# Remark: during any communication, the preset values are selected automatically according to the
# definition of bits in the RxModeReg and TxModeReg registers
CRCPreset_0000 = CRCPreset(0b00)
CRCPreset_6363 = CRCPreset(0b01)
CRCPreset_A671 = CRCPreset(0b10)
CRCPreset_FFFF = CRCPreset(0b11)

def make_ModeReg(crc_preset: CRCPreset, msb_first: bool=False, wait_for_field: bool=True, polarity_active_high: bool=True) -> bytes:
    """
    Make a register value for the ``ModeReg`` register

    :param CRCPreset crc_preset: which CRC preset to use for ``CalcCRC``, ignored during communication
    :param bool msb_first: CRC will be stored MSB first
    :param bool wait_for_field: Only start transmitting if RF field is active
    :param bool polarity_active_high: defines the polarity of pin MFIN
    :returns: single byte bytes object with register value
    """
    result = crc_preset
    if msb_first:
        result |= ModeReg_MSBFirst
    if wait_for_field:
        result |= ModeReg_TxWaitRF
    if polarity_active_high:
        result |= ModeReg_PolMFin
    return bytes([result])


# TxModeReg register: Defines the data rate during transmission

TxModeReg_TxCRCEn = (1 << 7)  # enables CRC generation during data transmission, can only be set to 0 at 106 kBd

# [6:4] : TxSpeed[2:0] defines the bit rate during data transmission
# These are reused for the RX Register
TrxSpeed_106kBd   = TrxSpeed(0b000)
TrxSpeed_212kBd   = TrxSpeed(0b001)
TrxSpeed_424kBd   = TrxSpeed(0b010)
TrxSpeed_848kBd   = TrxSpeed(0b011)

TxModeReg_InvMod  = (1 << 3)  # modulation of transmitted data is inverted

def make_TXModeReg(trxSpeed: TrxSpeed, invert_modulation: bool=False, enable_crc: bool=True) -> bytes:
    """
    Create value for ``TXModeReg`` register

    :param TrxSpeed trxSpeed: transmission speed
    :param bool invert_modulation: Invert modulation of transmitted data
    :param bool enable_crc: Enable CRC generation during transmission, can only be ``False`` for 106kBd
    :returns: single byte bytes object with register value
    """
    result = trxSpeed << 4
    if invert_modulation:
        result |= TxModeReg_InvMod
    if enable_crc:
        result |= TxModeReg_TxCRCEn
    return bytes([result])


# RxModeReg register: Defines the data rate during reception

RxModeReg_RxCRCEn    = (1 << 7)  # enables the CRC calculation during reception, can only be set to 0 at 106 kBd

# [6:4] : RxSpeed[2:0] defines the bit rate during data transmission
# These are reused from the TX Register

RxModeReg_RxNoErr    = (1 << 3)  # an invalid received data stream (less than 4 bits received) will be ignored and the receiver remains active

# unset: receiver is deactivated after receiving a data frame
# set: able to receive more than one data frame only valid for data rates above 106 kBd in order to handle the
#      polling command after setting this bit the Receive and Transceive commands will not terminate
#      automatically. Multiple reception can only be deactivated by writing any command (except the Receive
#      command) to the CommandReg register, or by the host clearing the bit
#
#      if set to 1, an error byte is added to the FIFO buffer at the end of a received data stream which
#      is a copy of the ErrorReg register value. For the MFRC522 version 2.0 the CRC status is reflected in
#      the signal CRCOk, which indicates the actual status of the CRC coprocessor. For the MFRC522 version
#      1.0 the CRC status is reflected in the signal CRCErr
RxModeReg_RxMultiple = (1 << 2)

def make_RXModeReg(trxSpeed: TrxSpeed, rx_multiple: bool=False, ignore_short_frames: bool=True, enable_crc: bool=True) -> bytes:
    """
    Create value for ``RXModeReg`` register

    :param TrxSpeed trxSpeed: transmission speed
    :param bool rx_multiple: Receive more than one data frame at once, used for polling, only above 106 kBd
    :param bool ignore_short_frames: Ignore short (< 4 bits) dataframes and continue receiving
    :param bool enable_crc: Enable CRC generation during transmission, can only be ``False`` for 106kBd
    :returns: single byte bytes object with register value
    """
    result = trxSpeed << 4
    if rx_multiple:
        result |= RxModeReg_RxMultiple
    if ignore_short_frames:
        result |= RxModeReg_RxNoErr
    if enable_crc:
        result |= RxModeReg_RxCRCEn
    return bytes([result])

# TxControlReg register: Controls the logical behavior of the antenna driver pins TX1 and TX2

TxControlReg_InvTx2RFOn  = (1 << 7)  # output signal on pin TX2 inverted when driver TX2 is enabled
TxControlReg_InvTx1RFOn  = (1 << 6)  # output signal on pin TX1 inverted when driver TX1 is enabled
TxControlReg_InvTx2RFOff = (1 << 5)  # output signal on pin TX2 inverted when driver TX2 is disabled
TxControlReg_InvTx1RFOff = (1 << 4)  # output signal on pin TX1 inverted when driver TX1 is disabled
TxControlReg_Tx2CW       = (1 << 3)  # set: output signal on pin TX2 continuously delivers the unmodulated 13.56 MHz energy carrier, unset: RF off
TxControlReg_Tx2RFEn     = (1 << 1)  # set: output signal on pin TX2 delivers the 13.56 MHz energy carrier modulated by the transmission data
TxControlReg_Tx1RFEn     = (1 << 0)  # set: output signal on pin TX1 delivers the 13.56 MHz energy carrier modulated by the transmission data


# TxASKReg register: Controls transmit modulation settings

TxASKReg_Force100ASK = (1 << 6)  # forces a 100 % ASK modulation independent of the ModGsPReg register setting


# TxSelReg register: Selects the internal sources for the analog module

# [5:4] : DriverSel[1:0] selects the input of drivers TX1 and TX2
TxDriver_Tristate         = TxDriver(0b00)  # 3-state; in soft power-down the drivers are only in 3-state mode if the DriverSel[1:0] value is set to 3-state mode
TxDriver_ModulationMiller = TxDriver(0b01)  # modulation signal (envelope) from the internal encoder, Miller pulse encoded
TxDriver_ModulationMFIN   = TxDriver(0b10)  # modulation signal (envelope) from pin MFIN
TxDriver_High             = TxDriver(0b11)  # HIGH; the HIGH level depends on the setting of bits InvTx1RFOn/InvTx1RFOff and InvTx2RFOn/InvTx2RFOff

# [3:0] : MFOutSel[3:0] selects the input for pin MFOUT
MFOut_Tristate            = MFOut(0b0000)  # 3-state
MFOut_Low                 = MFOut(0b0001)  # LOW
MFOut_High                = MFOut(0b0010)  # HIGH
MFOut_Test                = MFOut(0b0011)  # test bus signal as defined by the TestSel1Reg register’s TstBusBitSel[2:0] value
MFOut_ModulationMiller    = MFOut(0b0100)  # modulation signal (envelope) from the internal encoder, Miller pulse encoded
MFOut_Serial              = MFOut(0b0101)  # serial data stream to be transmitted, data stream before Miller encoder
MFOut_SerialManchester    = MFOut(0b0111)  # serial data stream received, data stream after Manchester decoder

def make_TxSelReg(driver: TxDriver, mfOut: MFOut=MFOut_Tristate) -> bytes:
    """
    Make value for ``TxSelReg`` register
    :param TxDriver driver: the driver to select
    :param MFOut mfOut: the input pin for MFOUT
    :returns: single byte bytes object with register value
    """
    result = driver << 4
    result |= mfOut
    return bytes([result])


# RxSelReg register: Selects internal receiver settings

# [7:6] : UARTSel[0:1] selects the input of the contactless UART
UARTSel_Low             = UARTSel(0b00)  # constant LOW
UARTSel_Manchester      = UARTSel(0b01)  # Manchester with subcarrier from pin MFIN
UARTSel_ModulatedAnalog = UARTSel(0b10)  # modulated signal from the internal analog module, default
UARTSel_NRZ             = UARTSel(0b11)  # NRZ coding without subcarrier from pin MFIN which is only valid for transfer speeds above 106 kBd

# [5:0] : RxWait[5:0] after data transmission the activation of the receiver is delayed for RxWait bit-clocks,
#         during this ‘frame guard time’ any signal on pin RX is ignored. This parameter is ignored by the
#         Receive command all other commands, such as Transceive, MFAuthent use this parameter
#         The counter starts immediately after the external RF field is switched on

def make_RxSelReg(rx_wait: int, uart: UARTSel=UARTSel_ModulatedAnalog) -> bytes:
    """
    Make value for ``RxSelReg`` register
    :param int rx_wait: delay receiver for rx_wait bit-clocks after transmission
    :param UARTSel uart: selects the input of the contactless UART
    :returns: single byte bytes object with register value
    """
    if rx_wait > 0b11111:
        raise ValueError('rx_wait maximum is 0b11111, it is {}'.format(rx_wait))
    result = rx_wait
    result |= (uart << 6)
    return bytes([result])


# RxThresholdReg register: Selects thresholds for the bit decoder

# [7:4] : MinLevel[3:0] defines the minimum signal strength at the decoder input that will be accepted
#         if the signal strength is below this level it is not evaluated

# [2:0] : CollLevel[2:0] defines the minimum signal strength at the decoder input that must be
#         reached by the weaker half-bit of the Manchester encoded signal to generate a bit-collision
#         relative to the amplitude of the stronger half-bit

def make_RxThresholdReg(min_level: int, coll_level: int) -> bytes:
    """
    Make value for ``RxThresholdReg`` register
    :param int min_level: minimum signal strength at the decoder input that will be accepted
    :param int coll_level: minimum signal strength at the decoder input that must be
                           reached by the weaker half-bit of the Manchester encoded signal
    :returns: single byte bytes object with register value
    """
    if min_level > 0b1111:
        raise ValueError('min_level to big, max is 0b1111, value is {}'.format(min_level))
    result |= min_level << 4
    if coll_level > 0b111:
        raise ValueError('coll_level to big, max is 0b111, value is {}'.format(coll_level))
    result |= coll_level
    return bytes([result])


# DemodReg register: Defines demodulator settings

DemodReg_FreezeStronger = (1 << 6)  # set: selects the stronger channel and freezes the selected channel during communication, unset: stronger channel
DemodReg_FixIQ          = (1 << 5)  # if AddIQ[1:0] are set to X0b, the reception is fixed to I channel else Q
DemodReg_TPrescalEven   = (1 << 4)  # on version 2.0: add one to the prescaler, default 0

# [3:2] : TauRcv[1:0] changes the time-constant of the internal PLL during data reception
#         Remark: if set to 00b the PLL is frozen during data reception

# [1:0] : TauSync[1:0] changes the time-constant of the internal PLL during burst

def make_DemodReg(freeze_channel: bool=False, fix_iq: bool=False, prescaler_even: bool=False, pll: int = 0, sync: int = 0) -> bytes:
    """
    Make value for ``DemodReg`` register
    :param bool freeze_channel: freeze stronger channel while receiving
    :param bool fix_iq: fix channel selection based on freeze_channel: 0 -> I, 1 -> Q
    :param bool prescaler_even: on version 2.0 fix the prescaler to an even value
    :param int pll: change time constant of PLL during reception, set to 0 to freeze PLL
    :param int sync: change time constant of PLL during burst
    :returns: single byte bytes object with register value
    """
    if sync > 0b11:
        raise ValueError('sync to big, max is 0b11, value is {}'.format(sync))
    result = sync
    if pll > 0b11:
        raise ValueError('pll to big, max is 0b11, value is {}'.format(pll))
    result |= (pll << 2)
    if prescaler_even:
        result |= DemodReg_TPrescalEven
    if fix_iq:
        result |= DemodReg_FixIQ
    if freeze_channel:
        result |= DemodReg_FreezeStronger
    return bytes([result])


# MfTxReg register: Controls some MIFARE communication transmit parameters.

# [1:0] : TxWait defines the additional response time, 7 bits are added to the value of the register bit by default

def makeMfTxReg(tx_wait: int) -> bytes:
    """
    Make value for ``MfTxReg`` register
    :param int tx_wait: additional response time
    :returns: single byte bytes object with register value
    """
    if tx_wait > 0b11:
        raise ValueError('tx_wait to big, max is 0b11, value is {}'.format(tx_wait))
    result = tx_wait
    return bytes([result])


# MfRxReg register

MfRxReg_ParityDisable = (1 << 4)


# SerialSpeedReg register

SerialSpeed_9600   = SerialSpeed(9600)    # -0.25 %
SerialSpeed_14400  = SerialSpeed(1440)    #  0.32 %
SerialSpeed_19200  = SerialSpeed(19200)   # -0.25 %
SerialSpeed_38400  = SerialSpeed(38400)   #  0.32 %
SerialSpeed_57600  = SerialSpeed(57600)   # -0.25 %
SerialSpeed_115200 = SerialSpeed(115200)  # -0.25 %
SerialSpeed_230400 = SerialSpeed(230400)  # -0.25 %
SerialSpeed_921600 = SerialSpeed(921600)  #  1.45 %

def make_SerialSpeedReg(speed: SerialSpeed) -> bytes:
    """
    Make value for ``SerialSpeedReg`` register
    :param SerialSpeed speed: speed to set
    :returns: single byte bytes object with register value
    """
    speed_table = {
        SerialSpeed_9600: 235,
        SerialSpeed_14400: 218,
        SerialSpeed_19200: 203,
        SerialSpeed_38400: 171,
        SerialSpeed_57600: 154,
        SerialSpeed_115200: 122,
        SerialSpeed_230400: 90,
        SerialSpeed_921600: 28
    }
    return bytes([speed_table[speed]])


# CRCResultReg registers: Shows the MSB and LSB values of the CRC calculation.

# Register 0x21 is MSB
# Register 0x22 is LSB


# ModWidthReg register: Sets the modulation width.

# [7:0] : ModWidth[7:0] defines the width of the Miller modulation as multiples of the carrier
#         frequency (ModWidth + 1 / fclk), the maximum value is half the bit period


# RFCfgReg register: Configures the receiver gain

# [6:4] : RxGain[2:0] defines the receiver’s signal voltage gain factor
RxGain_18dB = RxGain(0b000)
RxGain_23dB = RxGain(0b001)
#RxGain_18dB = RxGain(0b010)
#RxGain_23dB = RxGain(0b011)
RxGain_33dB = RxGain(0b100)
RxGain_38dB = RxGain(0b101)
RxGain_43dB = RxGain(0b110)
RxGain_48dB = RxGain(0b111)

def get_RxGain(rfCfgReg: int) -> RxGain:
    """
    Decode ``RFCfgReg`` register to return ``RxGain``
    :param int rfCfgReg: Register value to decode
    :returns: RxGain set in the register
    """
    return RxGain((rfCfgReg[0] & 0x70) >> 4)


def make_RFCfgReg(rx_gain: RxGain) -> int:
    """
    Make value for ``RFCfgReg`` register
    :param RxGain rx_gain: gain to set
    :returns: integer with register value
    """
    return rx_gain << 4

# GsNReg register: Defines the conductance of the antenna driver pins TX1 and TX2 for the n-driver when the driver is switched on

# TODO: GsNReg

# [7:4] : CWGsN[3:0] defines the conductance of the output n-driver during periods without modulation
#         which can be used to regulate the output power and subsequently current consumption and
#         operating distance
#         Remark: the conductance value is binary-weighted during soft Power-down mode the highest
#                 bit is forced to 1, value is only used if driver TX1 or TX2 is switched on


# [3:0] : ModGsN[3:0] defines the conductance of the output n-driver during periods without modulation
#         which can be used to regulate the modulation index
#         Remark: the conductance value is binary weighted during soft Power-down mode the highest
#                 bit is forced to 1, value is only used if driver TX1 or TX2 is switched on


# CWGsPReg register: Defines the conductance of the p-driver output during periods of no modulation.

# TODO: CWGsPReg

# [5:0] : CWGsP[5:0] defines the conductance of the p-driver output which can be used to regulate the
#         output power and subsequently current consumption and operating distance
#         Remark: the conductance value is binary weighted during soft Power-down mode the highest bit
#                 is forced to 1


# ModGsPReg register: Defines the conductance of the p-driver output during modulation

# TODO: ModGsPReg

# [5:0] : ModGsP[5:0] defines the conductance of the p-driver output during modulation which can be
#         used to regulate the modulation index
#         Remark: the conductance value is binary weighted during soft Power-down mode the highest bit
#                 is forced to 1, if the TxASKReg register’s Force100ASK bit is set to 1 the value
#                 of ModGsP has no effect


# TModeReg and TPrescalerReg registers: These registers define the timer settings.


# set: timer starts automatically at the end of the transmission in all communication modes at all speeds
#      if the RxModeReg register’s RxMultiple bit is not set, the timer stops immediately after receiving
#      the 5th bit (1 start bit, 4 data bits)
#      if the RxMultiple bit is set to 1 the timer never stops, in which case the timer can be
#      stopped by setting the ControlReg register’s TStopNow bit to 1
# unset: indicates that the timer is not influenced by the protocol
TModeReg_TAuto        = (1 << 7)

# [6:5] : TGated[1:0] internal timer is running in gated mode
#         Remark: in gated mode, the Status1Reg register’s TRunning bit is 1 when the timer is
#                 enabled by the TModeReg register’s TGated[1:0] bits this bit does not influence the
#                 gating signal
GatedMode_NonGated    = GatedMode(0b00)
GatedMode_MFIN        = GatedMode(0b01)
GatedMode_AUX1        = GatedMode(0b10)

# set: timer automatically restarts its count-down from the 16-bit timer reload value instead of counting
#      down to zero
# unset: timer decrements to 0 and the ComIrqReg register’s TimerIRq bit is set to 1
TModeReg_TAutoRestart = (1 << 4)

# [3:0] : TPrescaler_Hi[3:0] defines the higher 4 bits of the TPrescaler value

def _make_prescaler(frequency: int) -> int:
    return int((13.56 * 1000000) / frequency / 2 - 0.5)

def make_TModeReg(frequency: int, gated_mode: GatedMode=GatedMode_NonGated, auto_start: bool =True, auto_restart: bool =False) -> bytes:
    """
    Make value for ``TModeReg`` register
    :param int frequency: timer frequency
    :param GatedMode gated_mode: gated timer by external sources
    :param bool auto_start: Auto start timer
    :param bool auto_restart: Auto restart timer when it expired
    :returns: single byte bytes object with register value
    """
    result = 0
    if auto_start:
        result |= TModeReg_TAuto
    if auto_restart:
        result |= TModeReg_TAutoRestart
    result |= gated_mode << 5
    result |= _make_prescaler(frequency) >> 8
    return bytes([result])


def make_TPrescalerReg(frequency: int) -> bytes:
    """
    Make value for ``TPrescalerReg`` register
    :param int frequency: timer frequency, be sure to set ``TModeReg`` with the same frequency
    :returns: single byte bytes object with register value
    """
    return bytes([_make_prescaler(frequency) & 0xff])


# TReloadRegHi and TReloadRegLo registers: Defines the 16-bit timer reload value

# [7:0] : TReloadVal_Hi[7:0]
# [7:0] : TReloadVal_Lo[7:0]

# TCounterValRegHi and TCounterValRegLo registers: Contains the timer value

# [7:0] : TCounterVal_Hi[7:0]
# [7:0] : TCounterVal_Lo[7:0]


# TestSel1Reg register: General test signal configuration

# TODO: TestSel1Reg


# TestSel2Reg register: General test signal configuration and PRBS control.

# TODO: TestSel2Reg


# TestPinEnReg register: Enables the test bus pin output driver.

# TODO: TestPinEnReg


# TestPinValueReg register: Defines the HIGH and LOW values for the test port D1 to D7 when it is used as I/O

# TODO: TestPinValueReg


# TestBusReg register: Shows the status of the internal test bus.

# TODO: TestBusReg


# AutoTestReg register: Controls the digital self-test

# internal signal processing in the receiver chain is performed non-linearly which increases the
# operating distance in communication modes at 106 kBd
# Remark: due to non-linearity, the effect of the RxThresholdReg register’s MinLevel[3:0] and
#         the CollLevel[2:0] values is also non-linear
AutoTestReg_AmpRcv = (1 << 6)

# [3:0] : SelfTest[3:0] enables the digital self test
#         the self test can also be started by the CalcCRC command
#         the self test is enabled by value ``0b1001``
#         Remark: for default operation the self test must be disabled by value 0b0000


# VersionReg register

# [7:4] : Chiptype, 9 stands for MFRC522
# [3:0] : Version, can be 1 or 2

def get_version(versionReg: bytes) -> int:
    return versionReg[0] & 0x0f


# AnalogTestReg register: Determines the analog output test signal at, and status of, pins AUX1 and AUX2

# TODO: AnalogTestReg


# TestDAC1Reg register: Defines the test value for TestDAC1

# TODO: TestDAC1Reg


# TestDAC2Reg register: Defines the test value for TestDAC2

# TODO: TestDAC2Reg


# TestADCReg register: Shows the values of ADC I and Q channels

# TODO: TestADCReg

