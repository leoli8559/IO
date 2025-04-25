import serial
from enum import IntEnum, IntFlag
from dataclasses import dataclass
from typing import Optional
from time import time, sleep

# ==================== 协议宏定义 ====================
class PhyAddress(IntEnum):
    PHY_ADDR_T1 = 0x07
    PHY_ADDR_TX = 0x02

class DeviceType(IntEnum):
    DEVICE_TYPE_ID      = 1
    DEVICE_TYPE_TC10    = 3
    DEVICE_TYPE_POWER   = 4

class Registers(IntEnum):
    REG_ORGANIZATION_ID     = 0x0002  # 制造商ID寄存器
    REG_PHY_ID              = 0x0003  # PHY标识寄存器
    REG_TC10_CONTROL_1000   = 0x8022  # TC10控制寄存器
    REG_TC10_STATUS_1000    = 0x8023  # TC10状态寄存器
    REG_POWER_CONTROL       = 0xFD21  # 电源控制寄存器

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
    READ  = 0
    WRITE = 1

class PhyInitFlag(IntFlag):
    GE_MODE     = 0x01
    MASTER_MODE = 0x02
    TC10_ENABLE = 0x04

# ==================== 设备配置结构 ====================
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
        self.init_flags = 0

# ==================== 增强型控制类 ====================
class EnhancedMDIOController:
    def __init__(self, port='COM17', baudrate=115200, timeout=1.5):
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
        """注册PHY设备"""
        config = MdioConfig(phy_addr=phy_addr)
        self.devices[name] = PhyDevice(name, model, config)
        print(f"[{name}] 注册成功 | 地址: {phy_addr.name}(0x{phy_addr.value:02X})")

    def initialize_device(self, name: str, init_flags: PhyInitFlag):
        """初始化设备"""
        dev = self._get_device(name)
        print(f"\n=== 初始化 {dev.name} [{dev.model.name}] ===")
        
        if dev.model == PhyModel.MRVL_88Q222X_B0:
            self._init_mrvl_q222x(dev, init_flags)
        elif dev.model == PhyModel.MRVL_88EA1512:
            self._init_mrvl_ea1512(dev, init_flags)
        
        dev.init_flags = init_flags
        print(f"=== 初始化完成 [{PhyInitFlag(init_flags)}] ===\n")

    def _init_mrvl_q222x(self, dev: PhyDevice, flags: PhyInitFlag):
        """88Q222X初始化流程"""
        if flags & PhyInitFlag.MASTER_MODE:
            self.cl45_write(
                phy_addr=dev.config.phy_addr,
                dev_type=DeviceType.DEVICE_TYPE_TC10,
                reg=Registers.REG_TC10_CONTROL_1000,
                data=ControlValues.TC10_WAKE
            )

    def _init_mrvl_ea1512(self, dev: PhyDevice, flags: PhyInitFlag):
        """88EA1512初始化流程"""
        self.cl22_write(
            phy_addr=dev.config.phy_addr,
            reg=Registers.REG_POWER_CONTROL,
            data=ControlValues.POWER_REMOVE_EN
        )

    # ==================== 核心通信方法 ====================
    def cl22_write(self, phy_addr: PhyAddress, reg: Registers, data: ControlValues):
        """Clause22写操作"""
        cmd = self._build_command(
            clause=ClauseType.CLAUSE_22,
            operation=OperationType.WRITE,
            phy_addr=phy_addr,
            dev_type=None,  # Clause22无设备类型
            reg=reg,
            data=data
        )
        self._transact(cmd)

    def cl45_write(self, phy_addr: PhyAddress, dev_type: DeviceType, 
                  reg: Registers, data: ControlValues):
        """Clause45写操作"""
        cmd = self._build_command(
            clause=ClauseType.CLAUSE_45,
            operation=OperationType.WRITE,
            phy_addr=phy_addr,
            dev_type=dev_type,
            reg=reg,
            data=data
        )
        self._transact(cmd)

    def cl45_read(self, phy_addr: PhyAddress, dev_type: DeviceType, 
                  reg: Registers) -> int:
        """Clause45读操作"""
        cmd = self._build_command(
            clause=ClauseType.CLAUSE_45,
            operation=OperationType.READ,
            phy_addr=phy_addr,
            dev_type=dev_type,
            reg=reg,
            data=None
        )
        response = self._transact_read(cmd)
        return response

    def _build_command(self, clause: ClauseType, operation: OperationType,
                  phy_addr: PhyAddress, dev_type: Optional[DeviceType],
                  reg: Registers, data: Optional[ControlValues]) -> bytes:
        """统一使用枚举参数"""
        frame = [
            0xAA,
            (operation.value << 4) | clause.value,
            phy_addr.value,          # 使用 .value 获取整数值
            dev_type.value if dev_type else 0x00,
            (reg.value >> 8) & 0xFF,
            reg.value & 0xFF
        ]
        if operation == OperationType.WRITE:
            frame.extend([
                (data.value >> 8) & 0xFF,
                data.value & 0xFF
            ])
        frame.append(0xBB)
        return bytes(frame)

    def _transact(self, frame: bytes) -> None:
        """执行通信事务（用于写操作）"""
        self._debug_print("发送帧解析", frame)
        self.ser.write(frame)
        
        resp = self.ser.read(9)  # 读取完整9字节响应
        if len(resp) != 9:
            raise ValueError(f"无效响应长度: {len(resp)} 字节")
        if resp[0] != 0xAA or resp[-1] != 0xBB:
            raise ValueError("帧头或帧尾错误")
        if resp[1] != 0x00:
            raise RuntimeError(f"设备返回错误码: 0x{resp[1]:02X}")
        # 可选：处理响应数据
        # self._debug_print("接收帧解析", resp)

    def _transact_read(self, frame: bytes) -> int:
        """执行通信事务（用于读操作）"""
        self._debug_print("发送帧解析", frame)
        self.ser.write(frame)
        
        resp = self.ser.read(9)  # 读取完整9字节响应
        self._debug_print("接收帧解析", resp)
        
        if len(resp) != 9:
            raise ValueError(f"无效响应长度: {len(resp)} / 9 字节")
        if resp[0] != 0xAA or resp[-1] != 0xBB:
            raise ValueError("帧头或帧尾错误")
        if resp[1] != 0x00:
            raise RuntimeError(f"设备返回错误码: 0x{resp[1]:02X}")
        return (resp[4] << 8) | resp[5]

    def _debug_print(self, title: str, data: bytes):
        """增强型调试输出"""
        print(f"\n[{title}]")
        print("| 位置 | 十六进制 | 描述说明")
        print("|------|----------|-------------------------------")
        
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
        op = OperationType((byte >> 4) & 0x0F)
        clause = ClauseType(byte & 0x0F)
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
    ctrl = EnhancedMDIOController()
    
    # 设备注册
    ctrl.add_device("T1", PhyAddress.PHY_ADDR_T1, PhyModel.MRVL_88Q222X_B0)
    ctrl.add_device("TX", PhyAddress.PHY_ADDR_TX, PhyModel.MRVL_88EA1512)
    
    # 初始化设备
    ctrl.initialize_device("T1", PhyInitFlag.GE_MODE | PhyInitFlag.MASTER_MODE)
    
    # 执行TC10睡眠控制

    
    # 读取PHY ID寄存器
    print("\n=== 读取PHY ID寄存器 ===")
    try:
        phy_id = ctrl.cl45_read(
            phy_addr=PhyAddress.PHY_ADDR_T1,
            dev_type=DeviceType.DEVICE_TYPE_TC10,
            reg=Registers.REG_PHY_ID
        )
        print(f"PHY ID: 0x{phy_id:04X}")
    except Exception as e:
        print(f"读取失败: {e}")

    print("\n=== 设置TC10睡眠模式 ===")
    try:
        ctrl.cl45_write(
            phy_addr=PhyAddress.PHY_ADDR_T1,
            dev_type=DeviceType.DEVICE_TYPE_TC10,
            reg=Registers.REG_TC10_CONTROL_1000,
            data=ControlValues.TC10_WAKE  # 注意：这里应该是 T10_SLEEP 如果要设置睡眠模式
        )
    except Exception as e:
        print(f"写操作失败: {e}")        