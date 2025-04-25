import argparse
import sys
import serial
import time
from enum import IntEnum
from serial.tools import list_ports

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

# ================== 设备类型定义 ==================
class DeviceType(IntEnum):
    DEVICE_TYPE_ID = 0x01  # 设备类型 ID

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
RESPONSE_LENGTH = 9  # 帧数据部分长度（不包括回复编号）

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

    def _send_mdio_command(self, clause_type, operation, device_type, phy_addr, dev, reg, data=None):
        """
        构建并发送MDIO命令帧
        """
        # 控制字段：ClauseType (高4位) | OperationType (低4位)
        control_byte = ( operation.value<< 4) | clause_type.value

        # 构建发送帧
        cmd = bytearray()
        cmd.append(FRAME_START)  # 帧头 1   AA 21 07 01   00 02 00 00 BB
        cmd.append(control_byte)  # 控制字段  2
        #cmd.append(device_type.value)  # 设备类型  3
        cmd.append(phy_addr)  # PHY地址  4
        cmd.append(dev if dev is not None else 0x00)  # 设备编号（可选）
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
   

    # ========== 公开接口 ==========
    def cl22_write(self, phy_addr, reg_addr, data, device_type=DeviceType.DEVICE_TYPE_ID):
        """Clause22 写操作"""
        return self._send_mdio_command(
            clause_type=ClauseType.CLAUSE_22,
            operation=OperationType.WRITE,
            device_type=device_type,
            phy_addr=phy_addr,
            dev=None,  # Clause22 无设备编号
            reg=reg_addr,
            data=data
        )

    def cl22_read(self, phy_addr, reg_addr, device_type=DeviceType.DEVICE_TYPE_ID):
        """Clause22 读操作"""
        return self._send_mdio_command(
            clause_type=ClauseType.CLAUSE_22,
            operation=OperationType.READ,
            device_type=device_type,
            phy_addr=phy_addr,
            dev=None,  # Clause22 无设备编号
            reg=reg_addr,
            data=None  # 数据部分填充为 0x0000
        )

    def cl45_write(self, phy_addr, device, reg_addr, data, device_type=DeviceType.DEVICE_TYPE_ID):
        """Clause45 写操作"""
        return self._send_mdio_command(
            clause_type=ClauseType.CLAUSE_45,
            operation=OperationType.WRITE,
            device_type=device_type,
            phy_addr=phy_addr,
            dev=device,
            reg=reg_addr,
            data=data
        )

    def cl45_read(self, phy_addr, device, reg_addr, device_type=DeviceType.DEVICE_TYPE_ID):
        """Clause45 读操作"""
        return self._send_mdio_command(
            clause_type=ClauseType.CLAUSE_45,
            operation=OperationType.READ,
            device_type=device_type,
            phy_addr=phy_addr,
            dev=device,
            reg=reg_addr,
            data=None  # 数据部分填充为 0x0000
        )



    def get_phy_info(self, phy_addr, device_type=DeviceType.DEVICE_TYPE_ID):
        """获取PHY芯片信息"""
        try:
            org_id = self.cl45_read(phy_addr, 1, MRVL_ORGANIZATION_ID, device_type)
            if org_id != MRVL_ORGANIZATION_ID:
                raise ValueError("非Marvell芯片")
            
            aphy_id = self.cl45_read(phy_addr, 1, MRVL_APHY_ID, device_type)
            return {
                "model": (aphy_id & 0x3F0) >> 4,
                "revision": aphy_id & 0xF
            }
        except Exception as e:
            raise ValueError(f"信息获取失败: {str(e)}")







# ================== 命令行接口 ==================
def main():
    parser = argparse.ArgumentParser(description="工业级PHY控制终端")
    subparsers = parser.add_subparsers(dest='command')

    # 读取命令
    read_parser = subparsers.add_parser('read', help='读取寄存器')
    read_parser.add_argument('-t', '--type', choices=['22', '45'], required=True)
    read_parser.add_argument('-p', '--phy', type=int, default=0)
    read_parser.add_argument('-d', '--dev', type=int)
    read_parser.add_argument('-r', '--reg', type=lambda x: int(x, 0), required=True)
    read_parser.add_argument('-c', '--clause', choices=['22', '45'], required=True)

    # 写入命令
    write_parser = subparsers.add_parser('write', help='写入寄存器')
    write_parser.add_argument('-t', '--type', choices=['22', '45'], required=True)
    write_parser.add_argument('-p', '--phy', type=int, default=0)
    write_parser.add_argument('-d', '--dev', type=int)
    write_parser.add_argument('-r', '--reg', type=lambda x: int(x, 0), required=True)
    write_parser.add_argument('-v', '--value', type=lambda x: int(x, 0), required=True)
    write_parser.add_argument('-c', '--clause', choices=['22', '45'], required=True)

    # 信息命令
    subparsers.add_parser('info', help='获取PHY信息')

    args = parser.parse_args()
    mdio = MDIOController()

    try:
        # 映射子句类型
        clause_map = {'22': ClauseType.CLAUSE_22, '45': ClauseType.CLAUSE_45}

        if args.command == 'read':
            clause = clause_map[args.clause]
            if args.type == '22':
                val = mdio.cl22_read(args.phy, args.reg, device_type=DeviceType.DEVICE_TYPE_ID)
            else:
                val = mdio.cl45_read(args.phy, args.dev, args.reg, device_type=clause_map[args.clause])
            print(f"读取值: 0x{val:04X}")

        elif args.command == 'write':
            clause = clause_map[args.clause]
            if args.type == '22':
                mdio.cl22_write(args.phy, args.reg, args.value, device_type=DeviceType.DEVICE_TYPE_ID)
            else:
                mdio.cl45_write(args.phy, args.dev, args.reg, args.value, device_type=clause_map[args.clause])
            print("写入成功")

        elif args.command == 'info':
            info = mdio.get_phy_info(0, device_type=DeviceType.DEVICE_TYPE_ID)
            print(f"型号: 88Q222X-{info['model']:04X}")
            print(f"版本: Rev{info['revision']}")

    except ValueError as e:
        print(f"all操作失败: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()