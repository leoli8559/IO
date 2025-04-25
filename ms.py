import argparse
import sys
import serial
import time
from enum import IntEnum
# ///////////////////////////////////////////////////////////////////////////////////////////////
# ================== 硬件协议常量 ==================
MRVL_ORGANIZATION_ID = 0x2B
MRVL_APHY_ID = 0x0032

FRAME_START = 0xAA
FRAME_END = 0xBB

# ================== 子句定义 ==================
class ClauseType(IntEnum):
    CLAUSE_22 = 0
    CLAUSE_45 = 1

# ================== 操作码定义 ==================
class OperationType(IntEnum):
    WRITE = 0x01
    READ = 0x02

# ================== 错误码定义 ==================
class ErrorCode(IntEnum):
    SUCCESS = 0x00
    INVALID_FRAME = 0xFF
    INVALID_LENGTH = 0xFE
    INVALID_ADDRESS = 0xFD
    UNSUPPORTED_CLAUSE = 0xFC
    UNKNOWN_OPCODE = 0xFB

# ================== 串口配置 ==================
DEFAULT_PORT = "COM17"
BAUDRATE = 115200
TIMEOUT = 1.5
RESPONSE_LENGTH = 9  # 帧数据部分长度（包括回复编号，共 9 字节）

# ================== 核心控制类 ==================
class MDIOController:
    def __init__(self, port=DEFAULT_PORT, baudrate=BAUDRATE, timeout=TIMEOUT):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        self._validate_connection()

    def _validate_connection(self):
        if not self.ser.is_open:
            raise ConnectionError("串口连接失败，请检查硬件")

    def _send_mdio_command(self, clause_type, operation,     device_type_id, phy_addr, reg, data):
        """
        构建并发送MDIO命令帧
        """
        # 控制字段：ClauseType (高4位) | OperationType (低4位)
        control_byte = (  operation.value<< 4) |clause_type.value

        # 构建发送帧
        cmd = bytearray()
        cmd.append(FRAME_START)  # 帧头
        cmd.append(control_byte)  # 控制字段
        #cmd.append(device_type_id)  # 设备类型 ID（与设备编号相同）
        cmd.append(phy_addr)  # PHY地址
        cmd.append(device_type_id)  # 设备编号（与设备类型 ID 相同）
 
        cmd.append((reg >> 8) & 0xFF)  # 寄存器地址高位
        cmd.append(reg & 0xFF)  # 寄存器地址低位

        # 如果是写操作，添加数据部分；如果是读操作，填充为 0x0000
        if operation == OperationType.WRITE:
            if data is None:
                raise ValueError("写操作必须提供数据")
            cmd.append((data >> 8) & 0xFF)  # 数据高位
            cmd.append(data & 0xFF)  # 数据低位
        else:  # 读操作
            cmd.append(0x00)  # 数据高位填充
            cmd.append(0x00)  # 数据低位填充

        cmd.append(FRAME_END)  # 帧尾

        print(f"发送帧: {cmd.hex()}")
        
        self.ser.write(cmd)
        return self._read_response(operation, clause_type)

    def _read_response(self, expected_operation, expected_clause):
            """
            读取固定长度的响应帧，并处理超时
            """
            start_time = time.time()
            resp = bytearray()

            
            while True:
                if time.time() - start_time > TIMEOUT:
                    raise TimeoutError("通信超时: 未在指定时间内接收到完整响应")

                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting)
                    resp.extend(chunk)

                    # 检查是否接收到完整的帧
                    if len(resp) >= RESPONSE_LENGTH :  # 回复编号 + 帧数据
                        break

            # 检查响应帧的完整性和正确性
        # if len(resp) < RESPONSE_LENGTH + 1:
            #    raise ValueError(f"无效响应长度2: {len(resp)} / {RESPONSE_LENGTH + 1} 字节")
            
            print(f"  RS参数: frame={resp.hex()}")
            #self._debug_print("发送帧解析", resp)
            # 提取回复编号

            # 提取帧数据部分（去掉回复编号）
            frame_data = resp[0:]
            if len(frame_data) != RESPONSE_LENGTH:
                raise ValueError(f"无效帧数据长度: {len(frame_data)} / {RESPONSE_LENGTH} 字节")

            # 检查帧头和帧尾
            if frame_data[0] != FRAME_START or frame_data[-1] != FRAME_END:
                raise ValueError("帧头或帧尾错误")
            # 解析帧数据
            control_byte_received = frame_data[1]
            operation_received =frame_data[1]
            #clause_received = ClauseType((control_byte_received >> 4) & 0x0F)  #如果枚举类型异常，就会抛出错误到上层； 
            #operation_received = OperationType(control_byte_received & 0x0F)
            #device_type_received = DeviceType(frame_data[2])
            phy_addr_received = frame_data[2]
            device_received = frame_data[3]
            reg_high_received = frame_data[4]
            reg_low_received = frame_data[5]

            # 如果是读操作，提取数据部分
            if operation_received == 0:
                data_high_received = frame_data[6]
                data_low_received = frame_data[7]
                data_received = (data_high_received << 8) | data_low_received
                print(f"提取的数据部分: 0x{data_received:04X}")
                return data_received
                
            reply_code = resp[0+1]
            if reply_code != ErrorCode.SUCCESS.value:
                raise ValueError(f"XX操作失败: 回复编号 {reply_code:02X}")



            # 写操作无需返回数据
            return True

    def softResetGe(self, device_type_id=0x01):
        """
        执行 GE/1000Bmdio 软复位
        """
        # 启用 DCL 复位
        self._send_mdio_command(ClauseType.CLAUSE_45, OperationType.WRITE, 3, 7,  0xFE1B, 0x48)

        # 软复位
        self._send_mdio_command(ClauseType.CLAUSE_45, OperationType.WRITE, 3, 7, 0x0900, 0x8000)
        self._send_mdio_command(ClauseType.CLAUSE_45, OperationType.WRITE, 3, 7, 0xFFE4, 0x000C)

        # 禁用 DCL 复位
        self._send_mdio_command(ClauseType.CLAUSE_45, OperationType.WRITE, 3, 7,  0xFE1B, 0x58)

    def marvell_88q222x_setMasterSlave(self, device_type_id, forceMaster):
        """
        设置 PHY 的主从模式     def _send_mdio_command(self, clause_type, operation, device_type_id, phy_addr, reg, data=None):
        """
        if forceMaster:
            print("Set mode Master")
        else:
            print("Set mode Slave")

        data = 0x4000 if forceMaster else 0x0000
        self._send_mdio_command(ClauseType.CLAUSE_45, OperationType.WRITE, 1, 7, 0x0834, data)


    def direct_mdio_read(self, clause_type, device_type_id, phy_addr, reg_addr):
        """
        直接读取 MDIO 寄存器
        """
        return self._send_mdio_command(clause_type, OperationType.READ, device_type_id, phy_addr,  reg_addr,0)

    def direct_mdio_write(self, clause_type, device_type_id, phy_addr, reg_addr, data):
        """
        直接写入 MDIO 寄存器
        """
        return self._send_mdio_command(clause_type, OperationType.WRITE, device_type_id, phy_addr,  reg_addr, data)


# ================== 命令行接口 ==================
def main():
    parser = argparse.ArgumentParser(description="工业级PHY控制终端")
    subparsers = parser.add_subparsers(dest='command')

    # 软复位命令
    reset_parser = subparsers.add_parser('softReset', help='执行 GE/1000Bmdio 软复位')
    reset_parser.add_argument('--device', type=lambda x: int(x, 0), default=0x01, help='设备类型 ID (默认 0x01)')

    # 设置主从模式命令
    mode_parser = subparsers.add_parser('setMode', help='设置 PHY 的主从模式')
    mode_parser.add_argument('--master', action='store_true', help='设置为 Master 模式')
    mode_parser.add_argument('--slave', action='store_true', help='设置为 Slave 模式')
    mode_parser.add_argument('--device', type=lambda x: int(x, 0), default=0x01, help='设备类型 ID (默认 0x01)')

    # 直接读写命令
    direct_parser = subparsers.add_parser('direct', help='直接读写 MDIO 寄存器')
    direct_parser.add_argument('--clause', choices=['22', '45'], required=True, help='子句类型 (22 或 45)')
    direct_parser.add_argument('--type', choices=['read', 'write'], required=True, help='操作类型 (read 或 write)')
    direct_parser.add_argument('--device', type=lambda x: int(x, 0), required=True, help='设备类型 ID (单字节)')
    direct_parser.add_argument('--phy', type=int, required=True, help='PHY 地址')
    direct_parser.add_argument('--reg', type=lambda x: int(x, 0), required=True, help='寄存器地址')
    direct_parser.add_argument('--value', type=lambda x: int(x, 0), help='写入的值 (仅在写操作时使用)')

    args = parser.parse_args()
    mdio = MDIOController()

    try:
        # 映射子句类型
        clause_map = {'22': ClauseType.CLAUSE_22, '45': ClauseType.CLAUSE_45}

        if args.command == 'softReset':
            mdio.softResetGe(args.device)
            print("软复位完成")

        elif args.command == 'setMode':
            if args.master and not args.slave:
                mdio.marvell_88q222x_setMasterSlave(args.device, 1)
            elif args.slave and not args.master:
                mdio.marvell_88q222x_setMasterSlave(args.device, 0)
            else:
                print("错误: 请选择 --master 或 --slave")

        elif args.command == 'direct':
            clause = clause_map[args.clause]
            if args.type == 'read':
                value = mdio.direct_mdio_read(clause, args.device, args.phy, args.reg)
                print(f"Reg: 0x{args.reg:04X}")

                print(f"读取值: 0x{value:04X}")
            elif args.type == 'write':
                if args.value is None:
                    raise ValueError("写操作必须提供 --value 参数")
                mdio.direct_mdio_write(clause, args.device, args.phy, args.reg, args.value)
                print("写入成功")

        else:
            print("错误: 请选择有效的命令")

    except Exception as e:
        print(f"操作失败: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()