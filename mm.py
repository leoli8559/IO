import serial
from enum import IntEnum, IntFlag
from dataclasses import dataclass
from typing import Optional
from time import time, sleep
import time

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
        self._operation_timeout = timeout  # 操作超时时间
    
    def add_device(self, name: str, phy_addr: PhyAddress, model: PhyModel):
        """注册PHY设备"""
        print(f"[{self.__class__.__name__}.{self.add_device.__name__}] 开始执行")
        print(f"  参数: name={name}, phy_addr={phy_addr.name}(0x{phy_addr.value:02X}), model={model.name}")
        
        config = MdioConfig(phy_addr=phy_addr)  # 注意这里直接传递 phy_addr 枚举
        self.devices[name] = PhyDevice(name, model, config)
        print(f"  结果: 设备 {name} 注册成功 | 地址: {phy_addr.name}(0x{phy_addr.value:02X})")
        #print(f"[{self.__class__.__name__}.{self.add_device.__name__}] 执行完成")

    def _get_device(self, name: str) -> PhyDevice:
        """根据设备名称获取对应的 PhyDevice 对象"""
        print(f"[{self.__class__.__name__}.{self._get_device.__name__}] 开始执行")
        print(f"  参数: name={name}")
        
        if name not in self.devices:
            error_msg = f"设备 '{name}' 未注册"
            print(f"  错误: {error_msg}")
            raise ValueError(error_msg)
        
        device = self.devices[name]
        print(f"  结果: 找到设备 {name} | 模型: {device.model.name}, 地址: {device.config.phy_addr.name}(0x{device.config.phy_addr.value:02X})")
        print(f"[{self.__class__.__name__}.{self._get_device.__name__}] 执行完成")
        return device

    def initialize_device(self, name: str, init_flags: PhyInitFlag):
        """初始化设备"""
        print(f"[{self.__class__.__name__}.{self.initialize_device.__name__}] 开始执行")
        print(f"  参数: name={name}, init_flags={PhyInitFlag(init_flags)}")
        
        dev = self._get_device(name)
        print(f"  初始化 {dev.name} [{dev.model.name}]")
        
        if dev.model == PhyModel.MRVL_88Q222X_B0:
            self._init_mrvl_q222x(dev, init_flags)
        elif dev.model == PhyModel.MRVL_88EA1512:
            self._init_mrvl_ea1512(dev, init_flags)
        
        dev.init_flags = init_flags
        print(f"  初始化完成 [{PhyInitFlag(init_flags)}]")
        print(f"[{self.__class__.__name__}.{self.initialize_device.__name__}] 执行完成")

    def _init_mrvl_q222x(self, dev: PhyDevice, flags: PhyInitFlag):
        """88Q222X初始化流程"""
        print(f"[{self.__class__.__name__}.{self._init_mrvl_q222x.__name__}] 开始执行")
        print(f"  参数: dev={dev.name}, flags={PhyInitFlag(flags)}")
        
        if flags & PhyInitFlag.MASTER_MODE:
            self.cl45_write(
                phy_addr=dev.config.phy_addr,
                dev_type=DeviceType.DEVICE_TYPE_TC10,
                reg=Registers.REG_TC10_CONTROL_1000,
                data=ControlValues.TC10_WAKE
            )
        
        print(f"[{self.__class__.__name__}.{self._init_mrvl_q222x.__name__}] 执行完成")

    def _init_mrvl_ea1512(self, dev: PhyDevice, flags: PhyInitFlag):
        """88EA1512初始化流程"""
        print(f"[{self.__class__.__name__}.{self._init_mrvl_ea1512.__name__}] 开始执行")
        print(f"  参数: dev={dev.name}, flags={PhyInitFlag(flags)}")
        
        self.cl22_write(
            phy_addr=dev.config.phy_addr,
            reg=Registers.REG_POWER_CONTROL,
            data=ControlValues.POWER_REMOVE_EN
        )
        
        print(f"[{self.__class__.__name__}.{self._init_mrvl_ea1512.__name__}] 执行完成")

    # ==================== 核心通信方法 ====================
    def cl22_write(self, phy_addr: PhyAddress, reg: Registers, data: ControlValues):
        """Clause22写操作"""
        print(f"[{self.__class__.__name__}.{self.cl22_write.__name__}] 开始执行")
        print(f"  参数: phy_addr={phy_addr.name}(0x{phy_addr.value:02X}), reg={reg.name}(0x{reg.value:04X}), data={data.name}(0x{data.value:04X})")
        
        cmd = self._build_command(
            clause=ClauseType.CLAUSE_22,
            operation=OperationType.WRITE,
            phy_addr=phy_addr,
            dev_type=None,  # Clause22无设备类型
            reg=reg,
            data=data
        )
        self._transact(cmd)
        
        print(f"[{self.__class__.__name__}.{self.cl22_write.__name__}] 执行完成")

    def cl45_write(self, phy_addr: PhyAddress, dev_type: DeviceType, 
                  reg: Registers, data: ControlValues):
        """Clause45写操作"""
        print(f"[{self.__class__.__name__}.{self.cl45_write.__name__}] 开始执行")
        print(f"  参数: phy_addr={phy_addr.name}(0x{phy_addr.value:02X}), dev_type={dev_type.name}(0x{dev_type.value:02X}), reg={reg.name}(0x{reg.value:04X}), data={data.name}(0x{data.value:04X})")
        
        cmd = self._build_command(
            clause=ClauseType.CLAUSE_45,
            operation=OperationType.WRITE,
            phy_addr=phy_addr,
             dev_type=dev_type,
            reg=reg,
            data=data
        )
        self._transact(cmd)
        
        print(f"[{self.__class__.__name__}.{self.cl45_write.__name__}] 执行完成")

    def cl45_read(self, phy_addr: PhyAddress, dev_type: DeviceType, 
                  reg: Registers) -> int:
        """Clause45读操作"""
        print(f"[{self.__class__.__name__}.{self.cl45_read.__name__}] 开始执行")
        print(f"  参数: phy_addr={phy_addr.name}(0x{phy_addr.value:02X}), dev_type={dev_type.name}(0x{dev_type.value:02X}), reg={reg.name}(0x{reg.value:04X})")
        
        response = self._transact_read(cmd=self._build_command(
            clause=ClauseType.CLAUSE_45,
            operation=OperationType.READ,
            phy_addr=phy_addr,
            dev_type=dev_type,
            reg=reg,
            data=None
        ))

        print(f"  结果: 读取值=0x{response:04X}")
        
        print(f"[{self.__class__.__name__}.{self.cl45_read.__name__}] 执行完成")
        return response

    def _build_command(self, clause: ClauseType, operation: OperationType,
                  phy_addr: PhyAddress, dev_type: Optional[DeviceType],
                  reg: Registers, data: Optional[ControlValues]) -> bytes:
        """统一使用枚举参数"""
        print(f"[{self.__class__.__name__}.{self._build_command.__name__}] 开始构建命令")
        #print(f"  参数: clause={clause.name}, operation={operation.name}, phy_addr={phy_addr.name}(0x{phy_addr.value:02X}), reg={reg.name}(0x{reg.value:04X}), data={data.name(0x{data.value:04X}) if data else None}")
        
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
        print(f"  构建命令: {frame_bytes.hex()}")
        print(f"[{self.__class__.__name__}.{self._build_command.__name__}] 命令构建完成")
           # print(f"  构建命令: {frame_bytes.hex()}")  # 打印构建的完整帧
           #print(f"[{self.__class__.__name__}.{self._build_command.__name__}] 命令构建完成")

        return frame_bytes
    def _transact(self, frame: bytes) -> None:
        """执行通信事务（用于写操作）"""
        print(f"[{self.__class__.__name__}.{self._transact.__name__}] 开始执行")
        print(f"  参数: frame={frame.hex()}")
        self._debug_print("发送帧解析", frame)
        
        self.ser.write(frame)
        # 发送帧
        #self.ser.write(frame)
        print(f"  调试: 已发送帧: {frame.hex()}")  # 确认帧已发送
        #print("test")
              
        resp = bytearray()
        #print("test222")       
        start_time = time.time()  # 获取当前时间
        print(f"  调试: start_time={start_time}")  # 打印 start_time 的值
        
        expected_length = 9  # 预期响应长度
        
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            #print(f"  调试: 已过去的时间={elapsed_time:.2f}秒")  # 打印已过去的时间
            
            # 检查是否超时
            if elapsed_time > self._operation_timeout:
                error_msg = f"通信超时: 未在 {self._operation_timeout} 秒内接收到完整响应"
                print(error_msg)
                raise ValueError(error_msg)
            
            # 检查是否有数据可读
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                resp.extend(chunk)
                print(f"  接收到数据: {chunk.hex()}")
            
            # 添加一个退出条件，避免无限循环
            if len(resp) >= expected_length:
                break

        self._debug_print("收到的帧解析", resp)      
        print(f"  最终接收到的响应: {resp.hex()}")
        print(f"[{self.__class__.__name__}.{self._transact.__name__}] 执行完成")

    def _transactXX(self, frame: bytes) -> None:
        """执行通信事务（用于写操作）"""
        print(f"[{self.__class__.__name__}.{self._transact.__name__}] 开始执行")
        print(f"  参数: frame={frame.hex()}")
        
        self.ser.write(frame)
        
        resp = bytearray()
        start_time = time.time()  # 获取当前时间
        expected_length = 9  # 预期响应长度
        
        while True:
            if time.time() - start_time > self._operation_timeout:
                error_msg = f"通信超时: 未在 {self._operation_timeout} 秒内接收到完整响应"
                print(error_msg)
                raise ValueError(error_msg)
            
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                resp.extend(chunk)
                
                # 打印接收到的数据
                print(f"  接收到数据: {chunk.hex()}")
                
                # 检查是否接收到完整的帧
                if len(resp) >= expected_length:
                    break
        
        print(f"  最终接收到的响应: {resp.hex()}")
        
        if len(resp) != expected_length:
            raise ValueError(f"无效响应长度: {len(resp)} / {expected_length} 字节")
        if resp[0] != 0xAA or resp[-1] != 0xBB:
            raise ValueError("帧头或帧尾错误")
        if resp[1] != 0x00:
            raise RuntimeError(f"设备返回错误码: 0x{resp[1]:02X}")
        
        print(f"[{self.__class__.__name__}.{self._transact.__name__}] 执行完成")

    def _transact_read(self, frame: bytes) -> int:
        """执行通信事务（用于读操作）"""

        """执行通信事务（用于写操作）"""
        self._debug_print("发送帧解析", frame)
        print(f"[{self.__class__.__name__}.{self._transact_read.__name__}] 开始执行")
        print(f"  参数: frame={frame.hex()}")
        
        self.ser.write(frame)
        
        resp = bytearray()
        start_time = time.time()  # 获取当前时间
        expected_length = 9  # 预期响应长度
        
        while True:
            if time.time() - start_time > self._operation_timeout:
                error_msg = f"通信超时: 未在 {self._operation_timeout} 秒内接收到完整响应"
                print(error_msg)
                raise ValueError(error_msg)
            
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                resp.extend(chunk)
                
                # 检查是否接收到完整的帧
                if len(resp) >= expected_length:
                    break
        """执行通信事务（用于读操作）"""
        #self._debug_print("发送帧解析", frame)
        #self.ser.write(frame)
        
        #resp = self.ser.read(9)  # 读取完整9字节响应 ====================================================================
        self._debug_print("接收帧解析", resp)

        if len(resp) != 9:
            raise ValueError(f"无效响应长度: {len(resp)} / 9 字节")
        if resp[0] != 0xAA or resp[-1] != 0xBB:
            raise ValueError("帧头或帧尾错误")
        if resp[1] != 0x00:
            raise RuntimeError(f"设备返回错误码: 0x{resp[1]:02X}")
        return (resp[4] << 8) | resp[5]
                   
        print(f"  最终接收到的响应: {resp.hex()}")
        result = int.from_bytes(resp[4:6], byteorder='big')  # 假设返回的数据在帧的 4-5 字节
        print(f"  解析结果: {result:04X}")
        
        print(f"[{self.__class__.__name__}.{self._transact_read.__name__}] 执行完成")
        return result


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

# ==================== 主程序 ====================
if __name__ == "__main__":
    try:
        # 初始化控制器，设置串口和超时时间
        print("[主程序] 开始运行")
        ctrl = EnhancedMDIOController(port='COM17', baudrate=115200, timeout=2.0)
        
        # 注册PHY设备
        ctrl.add_device("T1", PhyAddress.PHY_ADDR_T1, PhyModel.MRVL_88Q222X_B0)
        ctrl.add_device("TX", PhyAddress.PHY_ADDR_TX, PhyModel.MRVL_88EA1512)
        
        # 初始化设备
        ctrl.initialize_device("T1", PhyInitFlag.GE_MODE | PhyInitFlag.MASTER_MODE)
        ctrl.initialize_device("TX", PhyInitFlag.GE_MODE | PhyInitFlag.MASTER_MODE)
        
        # 执行TC10睡眠模式设置
        print("=== EEE设置TC10睡眠模式 ===")
        ctrl.cl45_write(
            phy_addr=PhyAddress.PHY_ADDR_T1,
            dev_type=DeviceType.DEVICE_TYPE_TC10,
            reg=Registers.REG_TC10_CONTROL_1000,
            data=ControlValues.TC10_WAKE  # 如果要设置睡眠模式，请使用 ControlValues.TC10_SLEEP
        )
        
        # 读取PHY ID寄存器
        print("=== 读取PHY ID寄存器 ===")
        try:
            phy_id = ctrl.cl45_read(
                phy_addr=PhyAddress.PHY_ADDR_T1,
                dev_type=DeviceType.DEVICE_TYPE_TC10,
                reg=Registers.REG_TC10_CONTROL_1000
            )

            print(f"PHY ID: 0x{phy_id:04X}")
        except Exception as e:
            print(f"XX读取PHY ID失败: {e}")
        
        print("[主程序] 运行结束")
    except Exception as e:
        print(f"[主程序] 发生异常: {e}")