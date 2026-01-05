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
        self._operation_timeout = timeout  # 操作超时时间
    
    def add_device(self, name: str, phy_addr: PhyAddress, model: PhyModel):
        """注册PHY设备"""
        config = MdioConfig(phy_addr=phy_addr)  # 注意这里直接传递 phy_addr 枚举
        self.devices[name] = PhyDevice(name, model, config)
        print(f"[{name}] 注册成功 | 地址: {phy_addr.name}(0x{phy_addr.value:02X})")

    def _get_device(self, name: str) -> PhyDevice:
        """根据设备名称获取对应的 PhyDevice 对象"""
        if name not in self.devices:
            raise ValueError(f"设备 '{name}' 未注册")
        return self.devices[name]

    def _build_command(self, clause: ClauseType, operation: OperationType,
                  phy_addr: PhyAddress, dev_type: Optional[DeviceType],
                  reg: Registers, data: Optional[ControlValues]) -> bytes:
        """统一使用枚举参数"""
        # 确保 phy_addr 是 PhyAddress 枚举类型
        if not isinstance(phy_addr, PhyAddress):
            raise TypeError(f"phy_addr 必须是 PhyAddress 类型，但传入的是 {type(phy_addr)}")
        
        # 确保 dev_type 是 DeviceType 枚举类型（如果存在）
        if dev_type is not None and not isinstance(dev_type, DeviceType):
            raise TypeError(f"dev_type 必须是 DeviceType 类型，但传入的是 {type(dev_type)}")
        
        # 确保 reg 是 Registers 枚举类型
        if not isinstance(reg, Registers):
            raise TypeError(f"reg 必须是 Registers 类型，但传入的是 {type(reg)}")
        
        # 确保 data 是 ControlValues 枚举类型（如果存在）
        if data is not None and not isinstance(data, ControlValues):
            raise TypeError(f"data 必须是 ControlValues 类型，但传入的是 {type(data)}")
        
        # 构建帧
        frame = [
            0xAA,
            (operation.value << 4) | clause.value,
            phy_addr.value,  # 确保 phy_addr 是枚举类型
            dev_type.value if dev_type else 0x00,  # 确保 dev_type 是枚举类型
            (reg.value >> 8) & 0xFF,
            reg.value & 0xFF
        ]
        if operation == OperationType.WRITE:
            frame.extend([
                (data.value >> 8) & 0xFF,
                data.value & 0xFF
            ])
        frame.append(0xBB)
        frame_bytes = bytes(frame)
        self._debug_print("发送帧", frame_bytes)
        return frame_bytes



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
        self._operation_timeout = timeout  # 操作超时时间
    
    def _get_device(self, name: str) -> PhyDevice:
        """根据设备名称获取对应的 PhyDevice 对象"""
        if name not in self.devices:
            raise ValueError(f"设备 '{name}' 未注册")
        return self.devices[name]    
    def add_device(self, name: str, phy_addr: PhyAddress, model: PhyModel):
        """注册PHY设备"""
        config = MdioConfig(phy_addr=phy_addr.value)
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
            phy_addr.value,
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
        frame_bytes = bytes(frame)
        self._debug_print("发送帧", frame_bytes)
        return frame_bytes

    def _transact(self, frame: bytes) -> None:
        """执行通信事务（用于写操作）"""
        self.ser.write(frame)
        
        resp = bytearray()
        start_time = time.time()
        expected_length = 9  # 预期响应长度
        
        while True:
            if time.time() - start_time > self._operation_timeout:
                raise ValueError(f"通信超时: 未在 {self._operation_timeout} 秒内接收到完整响应")
            
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                resp.extend(chunk)
                
                # 检查是否接收到完整的帧
                while len(resp) >= expected_length:
                    # 假设帧头在缓冲区的起始位置
                    if resp[0] != 0xAA or resp[-1] != 0xBB:
                        # 移除无效的起始字节
                        resp.pop(0)
                        if not resp:
                            break
                        continue
                    if len(resp) == expected_length:
                        break
                    # 等待更多数据
                    break
                
                if len(resp) == expected_length:
                    break
        
        self._debug_print("接收帧", resp)
        
        if len(resp) != expected_length:
            raise ValueError(f"无效响应长度: {len(resp)} / {expected_length} 字节")
        if resp[0] != 0xAA or resp[-1] != 0xBB:
            raise ValueError("帧头或帧尾错误")
        if resp[1] != 0x00:
            raise RuntimeError(f"设备返回错误码: 0x{resp[1]:02X}")
        # 可选：处理响应数据
        # self._debug_print("接收帧解析", resp)

    def _transact_read(self, frame: bytes) -> int:
        """执行通信事务（用于读操作）"""
        self.ser.write(frame)
        
        resp = bytearray()
        start_time = time.time()
        expected_length = 9  # 预期响应长度
        
        while True:
            if time.time() - start_time > self._operation_timeout:
                raise ValueError(f"通信超时: 未在 {self._operation_timeout} 秒内接收到完整响应")
            
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                resp.extend(chunk)
                
                # 检查是否接收到完整的帧
                while len(resp) >= expected_length:
                    # 假设帧头在缓冲区的起始位置
                    if resp[0] != 0xAA or resp[-1] != 0xBB:
                        # 移除无效的起始字节
                        resp.pop(0)
                        if not resp:
                            break

if __name__ == "__main__":
    try:
        # 初始化控制器，设置串口和超时时间
        ctrl = EnhancedMDIOController(port='COM17', baudrate=115200, timeout=2.0)
        
        # 注册PHY设备
        ctrl.add_device("T1", PhyAddress.PHY_ADDR_T1, PhyModel.MRVL_88Q222X_B0)
        ctrl.add_device("TX", PhyAddress.PHY_ADDR_TX, PhyModel.MRVL_88EA1512)
        
        # 初始化设备
        ctrl.initialize_device("T1", PhyInitFlag.GE_MODE | PhyInitFlag.MASTER_MODE)
        ctrl.initialize_device("TX", PhyInitFlag.GE_MODE | PhyInitFlag.MASTER_MODE)
        
        # 执行TC10睡眠模式设置
        print("=== 设置TC10睡眠模式 ===")
        ctrl.cl45_write(
            phy_addr=PhyAddress.PHY_ADDR_T1,
            dev_type=DeviceType.DEVICE_TYPE_TC10,
            reg=Registers.REG_TC10_CONTROL_1000,
            data=ControlValues.TC10_WAKE  # 如果要设置睡眠模式，请使用 ControlValues.TC10_SLEEP
        )
        
        # 执行TC10唤醒模式设置（如果需要）
        # print("=== 设置TC10唤醒模式 ===")
        # ctrl.cl45_write(
        #     phy_addr=PhyAddress.PHY_ADDR_T1,
        #     dev_type=DeviceType.DEVICE_TYPE_TC10,
        #     reg=Registers.REG_TC10_CONTROL_1000,
        #     data=ControlValues.TC10_SLEEP
        # )
        
        # 读取PHY ID寄存器
        print("=== 读取PHY ID寄存器 ===")
        try:
            phy_id = ctrl.cl45_read(
                phy_addr=PhyAddress.PHY_ADDR_T1,
                dev_type=DeviceType.DEVICE_TYPE_TC10,
                reg=Registers.REG_PHY_ID
            )
            print(f"PHY ID: 0x{phy_id:04X}")
        except Exception as e:
            print(f"读取PHY ID失败: {e}")
        
        # 读取另一个PHY的ID寄存器
        print("=== 读取另一个PHY ID寄存器 ===")
        try:
            phy_id_tx = ctrl.cl45_read(
                phy_addr=PhyAddress.PHY_ADDR_TX,
                dev_type=DeviceType.DEVICE_TYPE_TC10,
                reg=Registers.REG_PHY_ID
            )
            print(f"PHY ID (TX): 0x{phy_id_tx:04X}")
        except Exception as e:
            print(f"读取PHY ID (TX)失败: {e}")
        
        # 示例：执行其他寄存器读写操作
        # 例如，读取或写入 T1_PHY_CTRL_REG
        print("=== 读取T1_PHY_CTRL_REG寄存器 ===")
        try:
            ctrl_reg = ctrl.cl45_read(
                phy_addr=PhyAddress.PHY_ADDR_T1,
                dev_type=DeviceType.DEVICE_TYPE_TC10,
                reg=Registers.REG_ORGANIZATION_ID  # 注意：根据实际寄存器地址调整
            )
            print(f"T1_PHY_CTRL_REG: 0x{ctrl_reg:04X}")
        except Exception as e:
            print(f"读取T1_PHY_CTRL_REG失败: {e}")
        
        # 写入寄存器示例（确保写入的值符合设备规范）
        print("=== 写入T1_PHY_CTRL_REG寄存器 ===")
        try:
            ctrl.cl45_write(
                phy_addr=PhyAddress.PHY_ADDR_T1,
                dev_type=DeviceType.DEVICE_TYPE_TC10,
                reg=Registers.REG_ORGANIZATION_ID,  # 注意：根据实际寄存器地址调整
                data=ControlValues.TC10_WAKE  # 示例数据，需根据实际需求调整
            )
            print("T1_PHY_CTRL_REG写入成功")
        except Exception as e:
            print(f"写入T1_PHY_CTRL_REG失败: {e}")
    
    except Exception as e:
        print(f"主程序运行时发生异常: {e}")
