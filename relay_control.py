import argparse
import sys
import serial
import time
from serial.tools import list_ports

# 协议常量定义
START_BYTE = 0xAA
END_BYTE = 0xBB
BAUDRATE = 115200  # 固定波特率
RESPONSE_LENGTH = 1  # 返回帧长度：AA 01 ST BOARD_ID BB

def list_serial_ports():
    """扫描并打印可用串口设备"""
    ports = list_ports.comports()
    print("\n[硬件检测] 可用串口列表：")
    if not ports:
        print("  * 未找到可用串口设备")
    else:
        for idx, port in enumerate(ports, 1):
            print(f"  {idx}. {port.device:8} | {port.description}")
    print("")

def build_command(board_id, relay_num, state):
    """构造控制命令帧"""
    if not 0x00 <= board_id <= 0xFE:
        raise ValueError("板卡ID需在0x00到0xFE之间")
    if not 1 <= relay_num <= 8:
        raise ValueError("继电器编号需在1到8之间")

    return bytes([
        START_BYTE,          # 起始字节
        0x01,                # 功能位
        relay_num - 1,       # 继电器编号（0基）
        0x00,                # 保留位
        0x01 if state else 0x00,  # 状态位
        0x00, 0x00, 0x00, 0x00, 0x00,  # 保留区域
        board_id,            # 板卡ID
        END_BYTE             # 结束字节
    ])

def parse_response(response, expected_board):
    """解析设备返回数据"""
    if len(response) != RESPONSE_LENGTH:
        raise ValueError(f"无效数据长度: {len(response)}字节")
    
    #if response[0] != START_BYTE or response[-1] != END_BYTE:
      #    raise ValueError("帧头/帧尾校验失败")
    
    board_id = response[0]
      #if board_id != expected_board:
        #  raise ValueError(f"板卡ID不匹配: 期望0x{expected_board:02X} 收到0x{board_id:02X}")
    
    return bool(board_id)  # 返回状态位

def send_serial_command(port, command, board_id, timeout=1.0, retries=3):
    """
    发送命令并接收响应
    :param port: 固定串口COM17
    :param command: 控制命令
    :param board_id: 目标板卡ID
    """
    for attempt in range(1, retries+1):
        try:
            with serial.Serial(
                port=port,
                baudrate=BAUDRATE,  # 固定波特率
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            ) as ser:
                ser.reset_input_buffer()
                ser.write(command)
                time.sleep(0.5)
                print(f"[第{attempt}次尝试] 指令已发送，等待响应...")
                
                response = ser.read(RESPONSE_LENGTH)
                print(response)
                if not response:
                    raise serial.SerialTimeoutException("设备未响应")
                    
                status = parse_response(response, board_id)
                
                return status
                
        except (serial.SerialException, ValueError) as e:
            print(f"通信失败: {str(e)}")
            if attempt < retries:
                print(f"{retries-attempt}次重试剩余...")
                time.sleep(0.5)
    
    raise RuntimeError(f"超过最大重试次数({retries})")

def main():
    list_serial_ports()
    
    # 配置命令行参数
    parser = argparse.ArgumentParser(
        description="工业继电器控制系统 - 正式版V3",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="使用示例：\n"
               "  python %(prog)s -b 0x00 -k 1 -s on\n"
               "  python %(prog)s -b 15 -k 3 -s off"
    )

    # 必需参数
    parser.add_argument('-b', '--board',
                       type=lambda x: int(x, 0),
                       required=True,
                       help='板卡ID (十六进制需加0x前缀，如0x00)')
    parser.add_argument('-k', '--relay',
                       type=int,
                       choices=range(1, 9),
                       required=True,
                       metavar='1-8',
                       help='继电器编号')
    parser.add_argument('-s', '--state',
                       choices=['on', 'off'],
                       required=True,
                       help='开关状态')

    # 可选参数
    parser.add_argument('-t', '--timeout',
                       type=float,
                       default=1.5,
                       help='响应超时时间（秒，默认1.5）')
    parser.add_argument('-r', '--retries',
                       type=int,
                       default=3,
                       help='最大重试次数（默认3）')

    args = parser.parse_args()

    try:
        port = "COM17"  # 固定串口
        print(f"[系统配置] 目标端口: {port} | 波特率: {BAUDRATE}")
        
        command = build_command(args.board, args.relay, args.state == 'on')
        print(f"[指令详情] 原始数据: {' '.join(f'{b:02X}' for b in command)}")
        
        result = send_serial_command(
            port=port,
            command=command,
            board_id=args.board,
            timeout=args.timeout,
            retries=args.retries
        )
        
        print(f"\n[最终状态] 操作{'成功' if result else '失败'}")
        sys.exit(0 if result else 1)
        
    except Exception as e:
        print(f"\n[系统错误] {str(e)}")
        sys.exit(2)

if __name__ == "__main__":
    main()