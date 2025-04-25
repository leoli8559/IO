import argparse
import sys
import serial
from enum import IntEnum
from time import time
from serial.tools import list_ports

# ================== 调试信息增强版 ==================
class MDIOController:
    def _send_mdio_command(self, clause_type, operation, phy_addr, dev, reg, data=None):
        """构造并发送MDIO命令帧"""
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
        
        # 显示发送序列（新增调试输出）
        print(f"[TX {time():.3f}] " + ' '.join(f'{b:02X}' for b in cmd))
        
        self.ser.write(cmd)
        return self._parse_response(cmd, operation)

    def _parse_response(self, sent_cmd, operation):
        """解析响应帧并显示接收序列"""
        start_time = time()
        resp = self.ser.read(8)
        elapsed = (time() - start_time) * 1000
        
        # 显示接收序列（新增调试输出）
        if resp:
            rx_str = ' '.join(f'{b:02X}' for b in resp)
        else:
            rx_str = "无响应"
        print(f"[RX +{elapsed:.1f}ms] {rx_str}")

        # 原有解析逻辑保持不变...
        # ...（校验和数据处理代码）

# ================== 协议说明文档 ==================
"""
MDIO控制协议规范（版本1.2）

1. 命令帧结构
------------------------------------------------
| 字节位置 | 名称       | 说明                 | 示例值
------------------------------------------------
| 0        | 起始位     | 固定0xAA            | AA
| 1        | 功能位     | [7-4]操作类型        | 
|          |            |   0: 读操作         |
|          |            |   1: 写操作         |
|          |            | [3-0]协议类型        |
|          |            |   0: Clause22       | 
|          |            |   1: Clause45       | 
| 2        | PHY地址    | 0-31                | 00
| 3        | 设备类型   | Clause45专用         | 03
| 4        | 寄存器高位 | 寄存器地址高8位       | 80
| 5        | 寄存器低位 | 寄存器地址低8位       | 23
| 6        | 数据高位   | 写操作时有效          | 00
| 7        | 数据低位   | 写操作时有效          | 01
| 8        | 结束位     | 固定0xBB            | BB

2. 响应帧结构
------------------------------------------------
| 字节位置 | 名称       | 说明                 | 有效值
------------------------------------------------
| 0        | 起始位     | 固定0xAA            | AA
| 1        | 状态码     | 0x00表示成功         | 00
| 2        | PHY地址    | 回显请求的PHY地址    | 00
| 3        | 设备类型   | 回显请求的设备类型    | 03
| 4        | 数据高位   | 读操作返回数据高位    | 80
| 5        | 数据低位   | 读操作返回数据低位    | 23
| 6        | 寄存器高位 | 回显请求的寄存器地址高 | 80
| 7        | 结束位     | 固定0xBB            | BB

3. 错误码表
---------------------------------
| 状态码 | 说明               
---------------------------------
| 0x00   | 操作成功           
| 0x01   | 无效的协议类型      
| 0x02   | 寄存器地址越界      
| 0x03   | 设备类型不支持      
| 0xFF   | 未知错误           

4. 通信示例
发送：AA 10 00 03 80 23 BB       (Clause45读操作)
接收：AA 00 00 03 12 34 80 BB     (成功返回0x1234)
"""