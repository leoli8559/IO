import argparse
import sys
import serial
from enum import IntEnum, IntFlag
from dataclasses import dataclass
from time import time
from typing import Union

# ==================== 协议宏定义 ====================
class PhyAddress(IntEnum):
    PHY_ADDR_T1 = 0x07
    PHY_ADDR_TX = 0x02

class DeviceType(IntEnum):
    DEVICE_TYPE_ID      = 1
    DEVICE_TYPE_TC10    = 3
    DEVICE_TYPE_POWER   = 4

class Registers(IntEnum):
    REG_ORGANIZATION_ID     = 0x0002
    REG_PHY_ID              = 0x0003
    REG_TC10_CONTROL_1000   = 0x8022
    REG_TC10_STATUS_1000    = 0x8023
    REG_POWER_CONTROL       = 0xFD21

class ControlValues(IntEnum):
    TC10_SLEEP      = 0x0001
    TC10_WAKE       = 0x0010
    POWER_REMOVE_EN = 0x0010

class PhyModel(IntEnum):
    MRVL_88Q222X_B0 = 0x0032
    MRVL_88EA1512   = 0x1512

class ClauseType(IntEnum):
    CLAUSE_22 = 0
    CLAUSE_45 = 1

class OperationType(IntEnum):
    READ = 0
    WRITE = 1

class PhyInitFlag(IntFlag):
    GE_MODE     = 0x01
    MASTER_MODE = 0x02
    TC10_ENABLE = 0x04

# ==================== 设备配置结构 ====================
@dataclass
class MdioConfig:
    phy_addr: PhyAddress  # 改为直接存储枚举实例
    mdc_port: int = 0
    mdc_pin: int = 0
    mdio_port: int = 0
    mdio_pin: int = 0

class PhyDevice:
    def __init__(self, name: str, model: PhyModel, config: MdioConfig):
        self.name = name
        self.model = model
        self.config = config
        self.init_flags = 0

# ==================== 核心控制类 ====================
class MDIOController:
    def __init__(self, port='COM7', baudrate=115200, timeout=1.5):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        self.devices = {}

    def add_device(self, name: str, phy_addr: PhyAddress, model: PhyModel):
        """修正：直接存储枚举实例"""
        config = MdioConfig(phy_addr=phy_addr)
        self.devices[name] = PhyDevice(name, model, config)
        print(f"[{name}] Registered | Address: {phy_addr.name}(0x{phy_addr.value:02X})")

    def initialize_device(self, name: str, init_flags: PhyInitFlag):
        dev = self._get_device(name)
        print(f"\n=== Initializing {dev.name} [{dev.model.name}] ===")
        
        if dev.model == PhyModel.MRVL_88Q222X_B0:
            self._init_mrvl_q222x(dev, init_flags)
        elif dev.model == PhyModel.MRVL_88EA1512:
            self._init_mrvl_ea1512(dev, init_flags)
        
        dev.init_flags = init_flags
        print(f"=== Initialization Complete [{PhyInitFlag(init_flags)}] ===\n")

    def _init_mrvl_q222x(self, dev: PhyDevice, flags: PhyInitFlag):
        """修正：传递枚举实例"""
        self.cl45_write(
            dev.config.phy_addr,
            DeviceType.DEVICE_TYPE_TC10,
            Registers.REG_TC10_CONTROL_1000,
            ControlValues.TC10_WAKE
        )

    def _init_mrvl_ea1512(self, dev: PhyDevice, flags: PhyInitFlag):
        self.cl22_write(
            dev.config.phy_addr,
            Registers.REG_POWER_CONTROL,
            ControlValues.POWER_REMOVE_EN
        )

    # ==================== 通信方法 ====================
    def cl22_write(self, phy_addr: PhyAddress, reg: Registers, data: ControlValues):
        cmd = self._build_cmd(
            clause=ClauseType.CLAUSE_22,
            operation=OperationType.WRITE,
            phy_addr=phy_addr,
            dev_type=None,  # Clause22无设备类型
            reg=reg,
            data=data
        )
        return self._execute_cmd(cmd)

    def cl45_write(self, phy_addr: PhyAddress, dev_type: DeviceType, 
                  reg: Registers, data: ControlValues):
        cmd = self._build_cmd(
            clause=ClauseType.CLAUSE_45,
            operation=OperationType.WRITE,
            phy_addr=phy_addr,
            dev_type=dev_type,
            reg=reg,
            data=data
        )
        return self._execute_cmd(cmd)

    def _build_cmd(self, clause: ClauseType, operation: OperationType,
                  phy_addr: PhyAddress, dev_type: Union[DeviceType, None],
                  reg: Registers, data: ControlValues) -> bytes:
        """统一使用枚举参数"""
        frame = [
            0xAA,
            (operation.value << 4) | clause.value,
            phy_addr.value,
            dev_type.value if dev_type else 0,
            (reg.value >> 8) & 0xFF,
            reg.value & 0xFF,
        ]
        if operation == OperationType.WRITE:
            frame.extend([
                (data.value >> 8) & 0xFF,
                data.value & 0xFF
            ])
        frame.append(0xBB)
        return bytes(frame)

    def _execute_cmd(self, cmd: bytes):
        self._print_debug("SEND", cmd)
        self.ser.write(cmd)
        
        resp = self.ser.read(8)
        if not resp:
            raise TimeoutError("Device timeout")
        
        self._print_debug("RECV", resp)
        return self._parse_resp(resp)

    def _parse_resp(self, resp: bytes):
        if len(resp) != 8:
            raise ValueError(f"Invalid length: {len(resp)} bytes")
        if resp[0] != 0xAA or resp[-1] != 0xBB:
            raise ValueError("Frame error")
        if resp[1] != 0x00:
            raise RuntimeError(f"Error: 0x{resp[1]:02X}")
        return (resp[4] << 8) | resp[5]

    # ==================== 调试工具 ====================
    def _print_debug(self, prefix: str, data: bytes):
        print(f"\n[{prefix}] {' '.join(f'{b:02X}' for b in data)}")
        print("| Pos | Hex  | Description")
        print("|-----|------|-------------")
        
        desc_map = {
            0: ("Start", "0xAA"),
            1: ("Control", self._parse_control(data[1])),
            2: ("PHY Addr", self._enum_desc(PhyAddress, data[2])),
            3: ("Dev Type", self._enum_desc(DeviceType, data[3])),
            4: ("Reg High", f"0x{data[4]:02X}"),
            5: ("Reg Low", f"0x{data[5]:02X}"),
            6: ("Data High", f"0x{data[6]:02X}") if len(data)>7 else "",
            7: ("Data Low", f"0x{data[7]:02X}") if len(data)>7 else "",
            -1: ("End", "0xBB")
        }
        
        for i, b in enumerate(data):
            desc = desc_map.get(i, ("", ""))
            print(f"| {i:3} | 0x{b:02X} | {desc[0]}: {desc[1]}")

    def _parse_control(self, byte: int) -> str:
        op = OperationType((byte >> 4) & 0x0F)
        clause = ClauseType(byte & 0x0F)
        return f"{op.name} | {clause.name}"

    def _enum_desc(self, enum_type, value: int) -> str:
        try:
            return f"{enum_type(value).name} (0x{value:02X})"
        except ValueError:
            return f"Unknown (0x{value:02X})"

    def _get_device(self, name: str) -> PhyDevice:
        if name not in self.devices:
            raise ValueError(f"Device not found: {name}")
        return self.devices[name]

# ==================== 使用示例 ====================
if __name__ == "__main__":
    ctrl = MDIOController()
    
    # 注册设备（使用枚举实例）
    ctrl.add_device("T1", PhyAddress.PHY_ADDR_T1, PhyModel.MRVL_88Q222X_B0)
    ctrl.add_device("Tx", PhyAddress.PHY_ADDR_TX, PhyModel.MRVL_88EA1512)
    
    # 初始化设备（传递枚举标志）
    ctrl.initialize_device("T1", PhyInitFlag.GE_MODE | PhyInitFlag.MASTER_MODE)
    #ctrl.cl45_write(0x07, 3, 0x8002, 0x0020)
 
    # 执行TC10控制（使用完整枚举链）
    print("\n=== Executing TC10 Control ===")
    ctrl.cl45_write(
        phy_addr=PhyAddress.PHY_ADDR_T1,
        dev_type=DeviceType.DEVICE_TYPE_TC10,
        reg=Registers.REG_TC10_CONTROL_1000,
        data=ControlValues.TC10_SLEEP
    )