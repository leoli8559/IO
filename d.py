import serial
from enum import IntEnum, IntFlag
from dataclasses import dataclass
from time import time
from typing import Union, Optional

# ==================== 协议宏定义 ====================
class PhyAddress(IntEnum):
    PHY_T1 = 0x07
    PHY_TX = 0x02

class DeviceType(IntEnum):
    IDENTIFICATION = 1
    TC10_CTRL = 3
    POWER_MGMT = 4

class Register(IntEnum):
    ORG_ID    = 0x0002  # 制造商ID寄存器
    PHY_ID     = 0x0003  # PHY标识寄存器
    TC10_CTRL = 0x8022  # TC10控制寄存器
    TC10_STAT = 0x8023  # TC10状态寄存器
    PWR_CTRL  = 0xFD21  # 电源控制寄存器

class CommandValue(IntEnum):
    TC10_SLEEP     = 0x0001
    TC10_WAKE      = 0x0010
    PWR_DISABLE    = 0x0000
    PWR_ENABLE     = 0x0010

class PhyModel(IntEnum):
    MRVL_88Q222X_B0 = 0x0032
    MRVL_88EA1512   = 0x1512

class Clause(IntEnum):
    CLAUSE_22 = 0
    CLAUSE_45 = 1

class Operation(IntEnum):
    READ  = 0
    WRITE = 1

class InitFlag(IntFlag):
    GE_MODE    = 0x01
    MASTER_MODE = 0x02
    TC10_ENABLED = 0x04

# ==================== 设备配置 ====================
@dataclass
class MdioConfig:
    phy_addr: PhyAddress
    mdc_port: int = 0
    mdc_pin: int = 0
    mdio_port: int = 0
    mdio_pin: int = 0

class PhyDevice:
    def __init__(self, name: str, model: PhyModel, config: MdioConfig):
        self.name = name
        self.model = model
        self.config = config
        self.status = 0

# ==================== 核心控制器 ====================
class EnhancedMDIO:
    def __init__(self, port: str = 'COM17', baudrate: int = 115200):
        self.interface = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1.5
        )
        self.devices = {}
        
    def register_device(self, name: str, addr: PhyAddress, model: PhyModel):
        """设备注册"""
        config = MdioConfig(phy_addr=addr)
        self.devices[name] = PhyDevice(name, model, config)
        print(f"[{name}] PHY @ {addr.name}(0x{addr.value:02X})")

    def initialize(self, device: str, flags: InitFlag):
        """设备初始化"""
        dev = self._get_device(device)
        print(f"\n--- Initializing {dev.name} ---")
        
        if dev.model == PhyModel.MRVL_88Q222X_B0:
            self._init_q222x(dev, flags)
        elif dev.model == PhyModel.MRVL_88EA1512:
            self._init_ea1512(dev, flags)
        
        dev.status = flags
        print(f"Init Flags: {bin(flags.value)}")

    def _init_q222x(self, dev: PhyDevice, flags: InitFlag):
        """88Q222X初始化流程"""
        if flags & InitFlag.MASTER_MODE:
            self.cl45_write(
                dev.config.phy_addr,
                DeviceType.TC10_CTRL,
                Register.TC10_CTRL,
                CommandValue.TC10_WAKE
            )

    def _init_ea1512(self, dev: PhyDevice, flags: InitFlag):
        """88EA1512初始化流程"""
        self.cl22_write(
            dev.config.phy_addr,
            Register.PWR_CTRL,
            CommandValue.PWR_ENABLE
        )

    # ==================== 通信核心 ====================
    def cl22_write(self, phy: PhyAddress, reg: Register, val: CommandValue):
        """Clause22写入操作"""
        frame = self._build_frame(
            Clause.CLAUSE_22,
            Operation.WRITE,
            phy,
            None,  # Clause22无设备类型
            reg,
            val
        )
        return self._transact(frame)

    def cl45_write(self, phy: PhyAddress, dev_type: DeviceType, 
                  reg: Register, val: CommandValue):
        """Clause45写入操作"""
        frame = self._build_frame(
            Clause.CLAUSE_45,
            Operation.WRITE,
            phy,
            dev_type,
            reg,
            val
        )
        return self._transact(frame)

    def _build_frame(self, clause: Clause, op: Operation, 
                    phy: PhyAddress, dev_type: Optional[DeviceType],
                    reg: Register, val: CommandValue) -> bytes:
        """协议帧构造器"""
        frame = [
            0xAA,  # 起始符
            (op.value << 4) | clause.value,
            phy.value,
            dev_type.value if dev_type else 0x00,
            (reg.value >> 8) & 0xFF,
            reg.value & 0xFF
        ]
        if op == Operation.WRITE:
            frame.extend([
                (val.value >> 8) & 0xFF,
                val.value & 0xFF
            ])
        frame.append(0xBB)  # 结束符
        return bytes(frame)

    def _transact(self, frame: bytes) -> bytes:
        """执行通信事务"""
        self._debug_print("发送帧解析", frame)
        self.interface.write(frame)
        
        resp = self.interface.read(9)  # 读取完整9字节响应
        self._debug_print("接收帧解析", resp)
        
        return self._validate_resp(resp)

    def _validate_resp(self, resp: bytes) -> bytes:
        """响应验证"""
        if len(resp) != 9:
            raise ValueError(f"无效响应长度: {len(resp)}/9字节")
        if resp[0] != 0xAA or resp[-1] != 0xBB:
            raise ValueError("帧边界错误")
        if resp[1] != 0x00:
            raise RuntimeError(f"设备错误码: 0x{resp[1]:02X}")
        return resp[4:6]  # 返回数据部分

    # ==================== 调试工具 ====================
    def _debug_print(self, title: str, data: bytes):
        """增强型调试输出"""
        print(f"\n[{title}]")
        print("| 位置 | 十六进制 | 解释说明")
        print("|------|----------|-------------------------------")
        
        # 字段解析器
        descriptors = [
            (0,  "帧头", lambda x: f"起始符 0xAA"),
            (1,  "控制位", self._parse_control),
            (2,  "PHY地址", lambda x: f"{PhyAddress(x).name} (0x{x:02X})"),
            (3,  "设备类型", self._parse_dev_type),
            (4,  "寄存器高位", lambda x: f"0x{x:02X}"),
            (5,  "寄存器低位", lambda x: f"0x{x:02X}"),
            (6,  "数据高位", lambda x: f"0x{x:02X}"),
            (7,  "数据低位", lambda x: f"0x{x:02X}"),
            (8,  "帧尾", lambda x: f"结束符 0xBB")
        ]
        
        for pos, byte in enumerate(data):
            desc = next((d for d in descriptors if d[0] == pos), None)
            if desc:
                _, field, parser = desc
                value = parser(byte) if callable(parser) else str(byte)
                print(f"| {pos:2}  |   0x{byte:02X}  | {field}: {value}")
            else:
                print(f"| {pos:2}  |   0x{byte:02X}  | 保留位")

    def _parse_control(self, byte: int) -> str:
        op = Operation((byte >> 4) & 0x0F)
        clause = Clause(byte & 0x0F)
        return f"{op.name} | {clause.name}"

    def _parse_dev_type(self, byte: int) -> str:
        try:
            return f"{DeviceType(byte).name} (0x{byte:02X})"
        except ValueError:
            return f"未知类型 (0x{byte:02X})"

    def _get_device(self, name: str) -> PhyDevice:
        if name not in self.devices:
            raise KeyError(f"未注册设备: {name}")
        return self.devices[name]

# ==================== 使用示例 ====================
if __name__ == "__main__":
    ctrl = EnhancedMDIO()
    
    # 设备注册
    ctrl.register_device("T1", PhyAddress.PHY_T1, PhyModel.MRVL_88Q222X_B0)
    ctrl.register_device("TX", PhyAddress.PHY_TX, PhyModel.MRVL_88EA1512)
    
    # 初始化设备
    ctrl.initialize("T1", InitFlag.GE_MODE | InitFlag.MASTER_MODE)
    
    # 执行TC10睡眠控制
    print("\n=== 设置TC10睡眠模式 ===")
    ctrl.cl45_write(
        PhyAddress.PHY_T1,
        DeviceType.TC10_CTRL,
        Register.TC10_CTRL,
        CommandValue.TC10_SLEEP
    )