
from typing import Union

from time import sleep
from spidev import SpiDev

from .rc522_register import *
from .selftest import VALID_SELFTESTS

class RC522:

    def register_write(self, register: RC522Register, data: bytes) -> None:
        raise NotImplementedError("This has to be overridden by subclasses")

    def register_write_byte(self, register: RC522Register, byte: int) -> None:
        self.register_write(register, bytes([byte]))

    def register_read(self, register: RC522Register, count: int=1) -> Union[int, bytes]:
        raise NotImplementedError("This has to be overridden by subclasses")
    
    def register_set(self, register: RC522Register, mask: int) -> None:
        value = self.register_read(register)
        new_value = value | mask
        if new_value != value:
            self.register_write_byte(register, new_value)

    def register_clear(self, register: RC522Register, mask: int) -> None:
        value = self.register_read(register)
        new_value = value & ~mask
        if new_value != value:
            self.register_write_byte(register, new_value)
    
    def execute_command(self, command: Command) -> None:
        self.register_write_byte(CommandReg, command)

    def flush_fifo(self) -> None:
        self.execute_command(Command_Idle)
        self.register_write_byte(FIFOLevelReg, FIFOLevelReg_FlushBuffer)

    def send_data(self, data) -> None:
        self.register_write(FIFODataReg, data)

    def calculate_crc(self, data: bytes) -> bytes:
        self.flush_fifo()
        
        # clear irq
        self.register_write_byte(DivIrqReg, DivIrqReg_CRCIRq)

        # fill fifo
        self.send_data(data)

        # Calculate CRC
        self.execute_command(Command_CalcCRC)

        # FIXME: Wait for interrupt instead of polling
        while True:
            value = self.register_read(DivIrqReg)
            if (value & DivIrqReg_CRCIRq) > 0:
                self.execute_command(Command_Idle)
                msb = self.register_read(CRCResultRegMSB)
                lsb = self.register_read(CRCResultRegLSB)
                return bytes([msb, lsb])
            sleep(0.01)

    def init(self) -> None:
        self.register_write(
            TxModeReg,
            make_TXModeReg(TrxSpeed_106kBd)
        )
        self.register_write(
            RxModeReg,
            make_RXModeReg(TrxSpeed_106kBd)
        )
        self.register_write_byte(ModWidthReg, 0x26)  # FIXME: Magic number

        # Watchdog timer, period 25us
        self.register_write(
            TModeReg,
            make_TModeReg(40000)
        )
        self.register_write(
            TPrescalerReg,
            make_TPrescalerReg(40000)
        )
        # preload 1000 -> 25ms
        self.register_write_byte(TReloadRegHi, 1000 >> 8)
        self.register_write_byte(TReloadRegLo, 1000 & 0xff)

        self.register_write_byte(TxASKReg, TxASKReg_Force100ASK)
        self.register_write(
            ModeReg,
            make_ModeReg(CRCPreset_6363)
        )
        self.antenna_state = True

    def reset(self, hard: bool=False) -> None:
        if hard:
            raise NotImplementedError("This has to be overridden by subclasses")
        else:
            self.execute_command(Command_SoftReset)
        sleep(0.05)  # 50 ms
        self.execute_command(Command_Idle)

    def selftest(self) -> bool:
        # reset and zero everything
        self.reset(hard=False)
        self.flush_fifo()
        self.send_data(bytes(25))
        self.execute_command(Command_Mem)

        self.register_write_byte(AutoTestReg, 0x09)  # enable selftest
        self.send_data(bytes(1))
        self.register_write_byte(CommandReg, Command_CalcCRC)

        # Poll fifo state
        for _ in range(50):
            value = self.register_read(FIFOLevelReg)
            if value >= 64:
                break
            sleep(0.02)

        self.execute_command(Command_Idle)
        data = self.register_read(FIFODataReg, count=64)
        self.register_write_byte(AutoTestReg, 0x00) # reset autotest

        for key, blob in VALID_SELFTESTS.items():
            if blob == data:
                print('Valid {}'.format(key))
                return True

        print(data)
        return False
        
    @property
    def antenna_state(self) -> bool:
        value = self.register_read(TxControlReg)
        return (value & (TxControlReg_Tx1RFEn | TxControlReg_Tx2RFEn)) > 0

    @antenna_state.setter
    def antenna_state(self, value: bool) -> None:
        if value:
            self.register_set(TxControlReg, TxControlReg_Tx1RFEn | TxControlReg_Tx2RFEn)
        else:
            self.register_clear(TxControlReg, TxControlReg_Tx1RFEn | TxControlReg_Tx2RFEn)
    
    @property
    def antenna_gain(self) -> RxGain:
        value = self.register_read(RFCfgReg)
        return get_RxGain(value)

    @antenna_gain.setter
    def antenna_gain(self, value: RxGain) -> None:
        self.register_clear(RFCfgReg,  make_RFCfgReg(RxGain_48dB))
        self.register_set(RFCfgReg,  make_RFCfgReg(value))

    @property
    def standby(self) -> bool:
        return (self.register_read(CommandReg) & CommandReg_PowerDown) > 0

    @standby.setter
    def standby(self, value: bool) -> None:
        if value:
            self.register_write_byte(CommandReg, Command_NoCmdChange | CommandReg_PowerDown)
        else:
            self.register_write_byte(CommandReg, Command_NoCmdChange)


