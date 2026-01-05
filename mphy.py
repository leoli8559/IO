import argparse
import sys
import serial
import time
from enum import IntEnum
from serial.tools import list_ports

# ================== 硬件协议常量 ==================
MRVL_ID_DEVICE = 1
MRVL_ORGANIZATION_ID_REG = 0x2
MRVL_ORGANIZATION_ID = 0x2B
MRVL_APHY_ID_REG = 0x3
MRVL_APHY_ID = 0x0032

# ================== 串口配置 ==================
DEFAULT_PORT = "COM8"
BAUDRATE = 115200
TIMEOUT = 1.5

# ================== MDIO协议 ==================
class ClauseType(IntEnum):
    CLAUSE_22 = 0
    CLAUSE_45 = 1

class MDIOError(Exception):
    """MDIO操作异常基类"""
    pass

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
            raise MDIOError("串口连接失败，请检查硬件")

    def _send_mdio_command(self, clause_type, operation, phy_addr, dev, reg, data=None):
        cmd = bytearray()
        cmd.append(0xAA)  # 帧头
        func_byte = (operation << 4) | clause_type
        cmd.append(func_byte)
        cmd.extend([
            phy_addr,
            dev,
            (reg >> 8) & 0xFF,
            reg & 0xFF,
        ])
        if operation == 1:  # 写操作
            cmd.extend([(data >> 8) & 0xFF, data & 0xFF])
        cmd.append(0xBB)  # 帧尾
        
        self.ser.write(cmd)
        return self._parse_response(operation)

    def _parse_response(self, operation):
        resp = self.ser.read(8)
        if len(resp) != 8:
            raise MDIOError("响应长度异常")
        if resp[0] != 0xAA or resp[-1] != 0xBB:
            raise MDIOError("帧校验失败")
        if resp[1] != 0x00:
            raise MDIOError(f"设备错误码: 0x{resp[1]:02X}")
        return (resp[4] << 8) | resp[5] if operation == 0 else True

    # ========== 公开接口 ==========
    def cl22_write(self, phy_addr, reg_addr, data):
        """Clause22 写操作"""
        self._send_mdio_command(
            ClauseType.CLAUSE_22, 1, phy_addr, 0, reg_addr, data)

    def cl22_read(self, phy_addr, reg_addr):
        """Clause22 读操作"""
        return self._send_mdio_command(
            ClauseType.CLAUSE_22, 0, phy_addr, 0, reg_addr)

    def cl45_write(self, phy_addr, device, reg_addr, data):
        """Clause45 写操作"""
        self._send_mdio_command(
            ClauseType.CLAUSE_45, 1, phy_addr, device, reg_addr, data)

    def cl45_read(self, phy_addr, device, reg_addr):
        """Clause45 读操作"""
        return self._send_mdio_command(
            ClauseType.CLAUSE_45, 0, phy_addr, device, reg_addr)

    def get_phy_info(self, phy_addr):
        """获取PHY芯片信息"""
        try:
            org_id = self.cl45_read(phy_addr, 1, MRVL_ORGANIZATION_ID_REG)
            if org_id != MRVL_ORGANIZATION_ID:
                raise MDIOError("非Marvell芯片")
            
            aphy_id = self.cl45_read(phy_addr, 1, MRVL_APHY_ID_REG)
            return {
                "model": (aphy_id & 0x3F0) >> 4,
                "revision": aphy_id & 0xF
            }
        except Exception as e:
            raise MDIOError(f"信息获取失败: {str(e)}")

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

    # 写入命令
    write_parser = subparsers.add_parser('write', help='写入寄存器')
    write_parser.add_argument('-t', '--type', choices=['22', '45'], required=True)
    write_parser.add_argument('-p', '--phy', type=int, default=0)
    write_parser.add_argument('-d', '--dev', type=int)
    write_parser.add_argument('-r', '--reg', type=lambda x: int(x, 0), required=True)
    write_parser.add_argument('-v', '--value', type=lambda x: int(x, 0), required=True)

    # 信息命令
    subparsers.add_parser('info', help='获取PHY信息')

    args = parser.parse_args()
    mdio = MDIOController()

    try:
        if args.command == 'read':
            if args.type == '22':
                val = mdio.cl22_read(args.phy, args.reg)
            else:
                val = mdio.cl45_read(args.phy, args.dev, args.reg)
            print(f"0x{val:04X}")

        elif args.command == 'write':
            if args.type == '22':
                mdio.cl22_write(args.phy, args.reg, args.value)
            else:
                mdio.cl45_write(args.phy, args.dev, args.reg, args.value)
            print("写入成功")

        elif args.command == 'info':
            info = mdio.get_phy_info(0)
            print(f"型号: 88Q222X-{info['model']:04X}\n版本: Rev{info['revision']}")

    except MDIOError as e:
        print(f"操作失败: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()