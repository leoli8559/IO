import argparse
import sys
import serial
import time
from enum import IntEnum

# ================== 2025 0421    硬件协议常量 ==================
MRVL_ORGANIZATION_ID = 0x2B
MRVL_APHY_ID = 0x0032

FRAME_START = 0xAA
FRAME_END = 0xBB

ETH_SPEED_100M   =  0x00
ETH_SPEED_1000M  =  0x01
ETH_SPEED_NA     =  0x03
Gcnt =0

# ================== 子句定义 ==================
class ClauseType(IntEnum):
    CLAUSE_22 = 0x0A
    CLAUSE_45 = 0x0B

# ================== 操作码定义 ==================
class OperationType(IntEnum):
    WRITE = 0x01
    READ =  0x02

# ================== 错误码定义 ==================
class ErrorCode(IntEnum):
    SUCCESS = 0x00
    INVALID_FRAME = 0xFF
    INVALID_LENGTH = 0xFE
    INVALID_ADDRESS = 0xFD
    UNSUPPORTED_CLAUSE = 0xFC
    UNKNOWN_OPCODE = 0xFB

# ================== 串口配置 ==================
DEFAULT_PORT = "COM6"
BAUDRATE = 9600
TIMEOUT = 1.5
RESPONSE_LENGTH = 9  # 帧数据部分长度（包括回复编号，共 9 字节）

# ================== 核心控制类 ============ // 前面插入 长度； 后面插入校验；========================================================================
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
            raise ConnectionError("串口连接失败，  请检查硬件")

    def _send_mdio_command(self, clause_type, operation, device_type_id, phy_addr,  reg, data=None):
        """
        构建并发送MDIO命令帧
        """
        #Gcnt +=1
        #time.sleep(0.1)
        # 控制字段：ClauseType (高4位) | OperationType (低4位)
        control_byte = ( operation.value<< 4) | clause_type.value

        # 构建发送帧
        cmd = bytearray()
        cmd.append(FRAME_START)  # 帧头 1   AA 21 07 01   00 02 00 00 BB
        cmd.append(7)  #lenth

        cmd.append(control_byte)  # 控制字段  2
        #cmd.append(device_type.value)  # 设备类型  3
        cmd.append(phy_addr)  # PHY地址  4
        cmd.append(device_type_id)  # 设备编号（可选）

        cmd.append((reg >> 8) & 0xFF)  # 寄存器地址高位
        cmd.append(reg & 0xFF)  # 寄存器地址低位

        # 如果是写操作，添加数据部分；如果是读操作，填充为 0x0000
        if operation == OperationType.WRITE:
            if data is None:
                raise ValueError("写操作必须提供数据")
            cmd.append((data >> 8) & 0xFF)  # 数据高位
            cmd.append(data & 0xFF)  # 数据低位

        else:  # 读操作
            cmd.append(0xfF)  # 数据高位填充
            cmd.append(0x00)  # 数据低位填充


        cmd.append(0x99)  #CS  校验和； ========================协议只是改变发送，接受的数据没有变化========================================

        cmd.append(FRAME_END)  # 帧尾
        
        print(f"  发送帧: {cmd.hex()}")
        self.ser.write(cmd)

        return        self._read_response(operation, clause_type)
    


    def _send_mdio_commandXX(self, clause_type, operation, device_type_id, phy_addr, reg, data=None):
        """
        构建并发送MDIO命令帧
        """
        control_byte = (clause_type.value << 4) | operation.value

        cmd = bytearray()
        cmd.append(FRAME_START)
        cmd.append(control_byte)
        #cmd.append(device_type_id)  # 设备类型 ID
        cmd.append(phy_addr)  # PHY 地址
        cmd.append(device_type_id)  # 设备编号（与设备类型 ID 相同）
        cmd.append((reg >> 8) & 0xFF)
        cmd.append(reg & 0xFF)

        if operation == OperationType.WRITE:
            if data is None:
                raise ValueError("写操作必须提供数据")
            cmd.append((data >> 8) & 0xFF)
            cmd.append(data & 0xFF)
        else:
            cmd.append(0x00)
            cmd.append(0x00)

        cmd.append(FRAME_END)
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
            
            #####################################################print(f"  RS参数: frame={resp.hex()}")
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


    def set_mdio_reg_whole(self, addr_dev, phy_addr, reg_num, val_in):
        """
        设置 MDIO 寄存器的值
        """
        self._send_mdio_command(ClauseType.CLAUSE_45, OperationType.WRITE, addr_dev, phy_addr, reg_num, val_in)

    def get_mdio_reg_whole(self, addr_dev, phy_addr, reg_num):  
        """
        获取 MDIO 寄存器的值
        """
        return self._send_mdio_command(ClauseType.CLAUSE_45, OperationType.READ, addr_dev, phy_addr, reg_num)






    def get_mdio_reg_whole_val_mask(self, addr_dev, phy_addr, reg_num, val_mask):
        # 读取当前寄存器值  void set_mdio_reg(Mdio *mdio,  uint8_t addr_dev, uint16_t reg_num, uint16_t val_mask, uint16_t val_in)
        # current_value = self._send_mdio_command(ClauseType.CLAUSE_45, OperationType.READ, addr_dev, phy_addr, reg_num)
        # 修改寄存器值
        #  new_value = (current_value & val_mask) | reg_num
        # 写回寄存器
        return val_mask & self._send_mdio_command(ClauseType.CLAUSE_45, OperationType.READ, addr_dev, phy_addr, reg_num)

    def Set_mdio_reg_whole_val_mask(self, addr_dev, phy_addr, reg_num, val_mask, val_in):

        # 读取当前寄存器值  void set_mdio_reg(Mdio *mdio,  uint8_t addr_dev, uint16_t reg_num, uint16_t val_mask, uint16_t val_in)
        current_value = self._send_mdio_command(ClauseType.CLAUSE_45, OperationType.READ, addr_dev, phy_addr, reg_num)

        # 修改寄存器值  self.Set_mdio_reg_whole_val_mask(ClauseType.CLAUSE_45, OperationType.WRITE, 1, 7, 0x0834, 0xbfff, data)
        new_value = (current_value & val_mask) | val_in

        # 写回寄存器
        return self._send_mdio_command(ClauseType.CLAUSE_45, OperationType.WRITE, addr_dev, phy_addr, reg_num, new_value)


    def marvell_88q222x_getMasterSlave(self, addr_dev, phy_addr):
        """
        获取当前的主从模式设置
        """
        reg_num = 0x8001 
        addr_dev =7

        value = self.get_mdio_reg_whole(addr_dev, phy_addr, reg_num)
        return (value >> 14) & 1

    def marvell_88q222x_getAnegEnabled(self, addr_dev, phy_addr):
        """
                    检查 Auto-Negotiation 是否启用
                    void set_mdio_reg(Mdio *mdio,  uint8_t addr_dev, uint16_t reg_num, uint16_t val_mask, uint16_t val_in)
            {
                uint16_t data =cl45_mdio_read(mdio, addr_dev, reg_num);

            //    data  = data & val_mask;
            //    data |= val_in;
                
                data  = (data & val_mask) | val_in;

                cl45_mdio_write(mdio, addr_dev, reg_num, data);

            }
            get_mdio_reg(mdio, 7, 0x0200, MRVL_88Q222X_AN_ENABLE) != 0

        """

        MRVL_88Q222X_AN_DISABLE=     0x0000
        MRVL_88Q222X_AN_RESET =      0x8000
        MRVL_88Q222X_AN_ENABLE=      0x1000

        MRVL_88Q222X_AN_RESTART=     0x0200
        MRVL_88Q222X_AN_COMPLETE=    0x0100
        reg_num = 0x0200
        value = 0
        #value = self.get_mdio_reg_whole_val_mask( 7, phy_addr, reg_num,MRVL_88Q222X_AN_ENABLE)
        return self.get_mdio_reg_whole_val_mask( 7, phy_addr, reg_num,MRVL_88Q222X_AN_ENABLE)!= 0

    def marvell_88q222x_getSpeed(self, addr_dev, phy_addr):
        """
        获取当前的数据速率   mdio.marvell_88q222x_getMasterSlave(addr_dev, phy_addr) 
        """
        if self.marvell_88q222x_getAnegEnabled(7, phy_addr):
            reg_num = 0x801A
            value = self.get_mdio_reg_whole_val_mask(7, phy_addr, reg_num,0x4000)
            return 1000 if (value >> 14) == 1 else 100
        else:
            reg_num = 0x0834
            value = self.get_mdio_reg_whole_val_mask(1, phy_addr, reg_num,0x0001)
            return 1000 if value else 100

    def marvell_88q222x_checkLink(self, addr_dev, phy_addr):
        """
        获取链路状态
        """
        speed = self.marvell_88q222x_getSpeed(addr_dev, phy_addr)
        if speed == 1000:
            reg_num_1 = 0x0901
            retData1 = self.get_mdio_reg_whole_val_mask(3, phy_addr, reg_num_1, 0x0004)
            retData1 = self.get_mdio_reg_whole_val_mask(3, phy_addr, reg_num_1, 0x0004)  # 读取两次: 链路低电平状态
            reg_num_2 = 0x8001
            retData2 = self.get_mdio_reg_whole_val_mask(7, phy_addr, reg_num_2, 0x3000)  # 本地和远程接收器状态
            reg_num_3 = 0xFD9D
            retData3 = self.get_mdio_reg_whole_val_mask(3, phy_addr, reg_num_3, 0x0010)  # 本地接收器状态 2
            return (retData1 == 0x0004) and (retData2 == 0x3000) and (retData3 == 0x0010)
        else:
            reg_num_1 = 0x8109
            retData1 = self.get_mdio_reg_whole_val_mask(3, phy_addr, reg_num_1, 0x0004)  # 链路
            reg_num_2 = 0x8108
            retData2 = self.get_mdio_reg_whole_val_mask(3, phy_addr, reg_num_2, 0x3000)  # 本地和远程接收器状态
            reg_num_3 = 0x8230
            retData3 = self.get_mdio_reg_whole_val_mask(3, phy_addr, reg_num_3, 0x0001)  # 解码器锁定状态
            return (retData1 == 0x0004) and (retData2 == 0x3000) and (retData3 == 0x0001)

    def marvell_88q222x_getRealTimeLinkStatus(self, addr_dev, phy_addr):
        """
        获取实时 PMA 链路状态
        """
        speed = self.marvell_88q222x_getSpeed(addr_dev, phy_addr)
        if speed == 1000:
            reg_num = 0x0901
            retData1 = self.get_mdio_reg_whole_val_mask(3, phy_addr, reg_num, 0x0004)
            retData1 = self.get_mdio_reg_whole_val_mask(3, phy_addr, reg_num, 0x0004)  # 读取两次: 寄存器低电平值
        else:
            reg_num = 0x8109

            retData1 = self.get_mdio_reg_whole_val_mask(3, phy_addr, reg_num, 0x0004)
        return retData1 != 0

    def marvell_88q222x_getSQIReading_8Level(self, addr_dev, phy_addr):
        """
        获取 8 级信号质量指示器 (SQI)  get_mdio_reg_whole_val_mask(self, addr_dev, phy_addr, reg_num, val_mask):
        """
        #if not self.marvell_88q222x_getRealTimeLinkStatus(addr_dev, phy_addr):
        #    return 0

        speed = self.marvell_88q222x_getSpeed(addr_dev, phy_addr)
        if speed == 100:
            regNum = 0x8230
            regVal = self.get_mdio_reg_whole_val_mask(3, phy_addr, regNum,0xE000)
            SQI = regVal >> 13
        else:
            self.set_mdio_reg_whole(3, phy_addr, 0xFCA6, 0x0016)  # 改变 SQI 偏移  get_mdio_reg_whole_val_mask
            #regNum = 0x8230
            regVal = self.get_mdio_reg_whole_val_mask(3, phy_addr, 0xFCD8, 0x0007 )
            SQI = regVal & 0x0007  # 提取 SQI 值  OK啦 7    ============================
        return SQI
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
        """  ClauseType.CLAUSE_45, OperationType.READ
        设置 PHY 的主从模式     def _send_mdio_command(self, clause_type, operation, device_type_id, phy_addr, reg, data=None):
        """
        if forceMaster:
            print("Set mode Master")
        else:
            print("Set mode Slave")
        phy_addr=7
        speed = self.marvell_88q222x_getSpeed(1, phy_addr)
        #if speed == 1000:   .marvell_88q222x_getMasterSlave(addr_dev, phy_addr)

        data = 0x4000 if forceMaster else 0x0000    

        """
        设置主从模式
        
        Args:
            force_master: 强制主模式标识（非零为Master，零为Slave）
            
        Returns:
            int: 操作结果（0表示成功）
        """
        #original_force = force_master
        
        if speed == 100:
            self.Set_mdio_reg_whole_val_mask( 1, phy_addr, 0x0834, 0xbfff, data)
            #normalized = self.MASTER_VALUE_MASTER if force_master else self.MASTER_VALUE_NORMAL  Set_mdio_reg_whole_val_mask

        else:
            #normalized = self.MASTER_VALUE_MASTER | 0x0001 if force_master else self.MASTER_VALUE_NORMAL
            self.Set_mdio_reg_whole_val_mask(1, phy_addr, 0x0834, 0xbfff, data)|0x0001
            # 执行软复位  softResetGe(self, device_type_id=0x01)
            self.softResetGe(1)

        # 写入寄存器
 
        
        #self.uart_send(f"Set mode {'Master' if force_master else 'Slave'}\r")
        return 0


    def set_speed(self, target_speed):
        """
        设置端口速率
        
        Args:
            target_speed: 目标速率（ETH_SPEED_1000M 或 ETH_SPEED_100M）
        """
        current_mode = self.marvell_88q222x_get_master_slave()
        
        if target_speed == self.ETH_SPEED_1000M:
            self.marvell_88q222x_init_ge(current_mode)
            self.uart_send("Set speed 1000M\r")
        else:
            self.marvell_88q222x_init_fe(current_mode)
            self.uart_send("Set speed 100M\r")

    def set_link_speed_and_mode(self, force_speed, op_mode):
        """
        设置速率和工作模式  speed = self.marvell_88q222x_getSpeed(addr_dev, phy_addr)
        
        Args:
            force_speed: 强制速率（ETH_SPEED_1000M/ETH_SPEED_100M）
            op_mode: 操作模式（MRVL_APHY_OP_MASTER/MRVL_APHY_OP_SLAVE）
        """
 
        MRVL_APHY_OP_SLAVE  = 0 
        MRVL_APHY_OP_MASTER = 1 
        MRVL_APHY_OP_NA     = 2 
        MRVL_APHY_OP_AUTO   = 3   
        phy_addr=7
        #speed = self.marvell_88q222x_getSpeed(1, phy_addr)  get_mdio_reg_whole(self, addr_dev, phy_addr, reg_num):
        reg_val =  self.get_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0x7,
            val_in=0x0834  # 设置LED闪烁模式   self.get_mdio_reg_whole(addr_dev=1,7,834)
        )
        
        force_speed = self.marvell_88q222x_getSpeed(1, 7)
        # 设置速率位（bit0）
        if force_speed == 1000:
            reg_val = (reg_val & 0xFFF0) | 0x0001
        else:
            reg_val = (reg_val & 0xFFF0) | 0x0000
        
        # 设置操作模式（bit14-15）
        if op_mode == MRVL_APHY_OP_MASTER:
            reg_val |= 0x4000
        else:
            reg_val &= 0xBFFF
        
        self.set_mdio_reg_whole(
            addr_dev=1,
            phy_addr=7,
            reg_num=0x0834,
            val_in=reg_val
        )

    def marvell_88q222x_set_speed(self, lp_speed):
        """
        设置PHY工作速率
        
        Args:
            lp_speed: 目标速率（ETH_SPEED_1000M 或 ETH_SPEED_100M）
            
        Returns:
            bool: 操作成功状态    marvell_88q222x_set_speed(ETH_SPEED_1000M)
        """
        # 获取当前主从模式
        master_slave = self.marvell_88q222x_getMasterSlave(1, 7)  #marvell_88q222x_get_master_slave()
        
        try:
            if lp_speed == ETH_SPEED_1000M:
                self.marvell_88q222x_init_ge(master_slave)
                #self.uart_send("                                                                                    Set speed 1000M\r")

            elif lp_speed == ETH_SPEED_100M:
                self.marvell_88q222x_init_fe(master_slave)
                #self.uart_send("                                                                                    Set speed 100M\r ")
            else:
                raise ValueError("Invalid speed parameter")
                
            return True
        
        except Exception as e:
            # self.uart_send(f"Speed set failed: {str(e)}\r")
            print("                                                   输出Exception  ")
            return False

    def marvell_88q222x_init_fe(self, op_mode):
        """
        初始化100BASE-T1模式
        
        Args:
            op_mode: 操作模式（MRVL_APHY_OP_MASTER/MRVL_APHY_OP_SLAVE）
        """
        #print("    marvell_88q222x_init_fe                                                       LED配置")
       # if not self.marvell_88q222x_is_timeout_accessible():
          #  return

        #                     self.apply_fe_init()
        self.set_link_speed_and_mode(ETH_SPEED_100M, op_mode)
        print("                                                                                     LED配置")
        # LED配置
        self.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=7,
            reg_num=0x8000,
            val_in=0xFFF7  # 设置LED引脚模式
        )
        self.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=7,
            reg_num=0x8310,
            val_in=0x0030  # 激活链路活动LED
        )
        self.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=7,
            reg_num=0x8016,
            val_in=0x0030  # 设置LED闪烁模式
        )
        print("设置LED闪烁模式")
        self.soft_reset_fe()

    def marvell_88q222x_init_ge(self, op_mode):
        """
        初始化1000BASE-T1模式
        
        Args:
            op_mode: 操作模式（MRVL_APHY_OP_MASTER/MRVL_APHY_OP_SLAVE）
        """
        #if not self.marvell_88q222x_is_timeout_accessible():
        #    return

        #             self.apply_ge_init()
        print("  marvell_88q222x_init_gGGe                                                LED配置")
        self.set_link_speed_and_mode(ETH_SPEED_1000M, op_mode)
        
        # LED配置   set_mdio_reg_whole(self, addr_dev, phy_addr, reg_num, val_in)
        self.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=7,
            reg_num=0x8000,
            val_in=0xFFF7  # 设置LED引脚模式
        )

        self.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=7,
            reg_num=0x8310,
            val_in=0x0030  # 激活链路活动LED
        )
        self.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=7,
            reg_num=0x8016,
            val_in=0x0030  # 设置LED闪烁模式
        )
        
        # CRC计数器配置
        self.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=7,
            reg_num=0xFD07,
            val_in=0x0001  # 使能计数器
        )
        
        # RGMII时序配置
        self.set_mdio_reg_whole(
            addr_dev=4,
            phy_addr=7,
            reg_num=0xA001,
            val_in=0x0000  # 输入时钟无延迟
        )
        self.set_mdio_reg_whole(
            addr_dev=4,
            phy_addr=7,
            reg_num=0xA001,
            val_in=0x0000  # 输出时钟无延迟
        )
        print("                               输出时钟无延迟")
        self.soft_reset_ge()


 
 

    def soft_reset_ge(self):
        """执行千兆模块软复位"""
           #self.mdio.set_register(1, 0x0834, 0xFFFF, 0x0000)
        self.uart_send("GE Soft Reset\r")

    def uart_send(self, message):
        """通用UART发送接口"""
        # 实际实现需要连接硬件UART
        print(f"[UART] {message}")




    def marvell_88ea1512_getID(self, addr_dev, phy_addr, device_type_id=0):
        """
        获取当前的主从模式设置
        """
        reg_num = 0x2
       # addr_dev =7

        value = self._send_mdio_command(
                ClauseType.CLAUSE_22,
                OperationType.READ,
                device_type_id,
                phy_addr,
                reg_num,          # 寄存器地址
                0x000      # 软复位 + 强制100Mbps全双工
            )  
        return value



    def marvell_88ea1512_setSpeed_fe(self, phy_addr=2, device_type_id=0):
        """
        设置百兆模式（C22协议实现）
        Args:
            phy_addr: PHY地址，默认0x07
            device_type_id: 设备类型ID，默认0
        """
        PAGE_REG = 22  # 页面选择寄存器
        
        try:
            self._send_mdio_command(
                ClauseType.CLAUSE_22,
                OperationType.WRITE,
                device_type_id,
                phy_addr,
                PAGE_REG,          # 寄存器地址
                0x000      # 软复位 + 强制100Mbps全双工
            )           
            # 1. 软复位PHY（页0寄存器0）
            self._send_mdio_command(
                ClauseType.CLAUSE_22,
                OperationType.WRITE,
                device_type_id,
                phy_addr,
                0,          # 寄存器地址
                0x8000      # 软复位 + 强制100Mbps全双工
            )

            # 2. 配置自动选择主从/全双工（寄存器9）
            self._send_mdio_command(
                ClauseType.CLAUSE_22,
                OperationType.WRITE,
                device_type_id,
                phy_addr,
                9,          # 寄存器地址
                0x0000      # autoneg_disable + full_duplex
            )

            # 3. 配置LED状态指示（页3寄存器16）
            self._send_mdio_command(
                ClauseType.CLAUSE_22,
                OperationType.WRITE,
                device_type_id,
                phy_addr,
                PAGE_REG,   # 切换到页3
                3          # 寄存器地址
            )
            self._send_mdio_command(
                ClauseType.CLAUSE_22,
                OperationType.WRITE,
                device_type_id,
                phy_addr,
                16,         # 寄存器地址
                0x1034      # LED模式配置
            )


            # 4. 设置MAC层100Mbps（页2寄存器21）
            self._send_mdio_command(
                ClauseType.CLAUSE_22,
                OperationType.WRITE,
                device_type_id,
                phy_addr,
                PAGE_REG,   # 切换到页2
                2          # 寄存器地址
            )
            self._send_mdio_command(
                ClauseType.CLAUSE_22,
                OperationType.WRITE,
                device_type_id,
                phy_addr,
                21,         # 寄存器地址
                0x3006      # 100Mbps + full_duplex
            )

            # 5. 锁定PHY速率（页0寄存器0）
            self._send_mdio_command(
                ClauseType.CLAUSE_22,
                OperationType.WRITE,
                device_type_id,
                phy_addr,
                PAGE_REG,   # 切换到页0
                0           # 寄存器地址
            )
            self._send_mdio_command(
                ClauseType.CLAUSE_22,
                OperationType.WRITE,
                device_type_id,
                phy_addr,
                0,          # 寄存器地址
                0xa100      # 强制锁定100Mbps全双工
            )

            print("C22协议百兆模式设置完成")
            return ErrorCode.SUCCESS.value

        except Exception as e:
            print(f"设置失败: {str(e)}")
            return ErrorCode.UNKNOWN_OPCODE.value
    def marvell_88ea1512_setSpeed_1000M(self, phy_addr=2, device_type_id=0):
            """
            设置1000兆模式（C22协议实现）
            Args:
                phy_addr: PHY地址，默认0x07
                device_type_id: 设备类型ID，默认0
            """
            PAGE_REG = 22  # 页面选择寄存器
            
            try:
                self._send_mdio_command(
                    ClauseType.CLAUSE_22,
                    OperationType.WRITE,
                    device_type_id,
                    phy_addr,
                    PAGE_REG,   # 切换到页3
                    3          # 寄存器地址
                )            
                # 1. 软复位PHY（页0寄存器0）
                self._send_mdio_command(
                    ClauseType.CLAUSE_22,
                    OperationType.WRITE,
                    device_type_id,
                    phy_addr,
                    0x10,          # 寄存器地址
                    0x1012      # 软复位 + 强制100Mbps全双工
                )

                self._send_mdio_command(
                    ClauseType.CLAUSE_22,
                    OperationType.WRITE,
                    device_type_id,
                    phy_addr,
                    PAGE_REG,          # 寄存器地址
                    0x12      # 软复位 + 强制100Mbps全双工
                )
                # 2. 配置自动选择主从/全双工（寄存器9）
                self._send_mdio_command(
                    ClauseType.CLAUSE_22,
                    OperationType.WRITE,
                    device_type_id,
                    phy_addr,
                    0x14,          # 寄存器地址
                    0x0000      # autoneg_disable + full_duplex
                )
                self._send_mdio_command(
                    ClauseType.CLAUSE_22,
                    OperationType.WRITE,
                    device_type_id,
                    phy_addr,
                    0x14,          # 寄存器地址
                    0x8000      # autoneg_disable + full_duplex
                )

                # 3. 配置LED状态指示（页3寄存器16）
                self._send_mdio_command(
                    ClauseType.CLAUSE_22,
                    OperationType.WRITE,
                    device_type_id,
                    phy_addr,
                    PAGE_REG,   # 切换到页3
                    2          # 寄存器地址
                )
                self._send_mdio_command(
                    ClauseType.CLAUSE_22,
                    OperationType.WRITE,
                    device_type_id,
                    phy_addr,
                    0x15,         # 寄存器地址
                    0x1046      # LED模式配置
                )

                # 4. 设置MAC层100Mbps（页2寄存器21）
                self._send_mdio_command(
                    ClauseType.CLAUSE_22,
                    OperationType.WRITE,
                    device_type_id,
                    phy_addr,
                    PAGE_REG,   # 切换到页2
                    0          # 寄存器地址
                )
                self._send_mdio_command(
                    ClauseType.CLAUSE_22,
                    OperationType.WRITE,
                    device_type_id,
                    phy_addr,
                    9,         # 寄存器地址
                    0x300      # 100Mbps + full_duplex
                )


                self._send_mdio_command(
                    ClauseType.CLAUSE_22,
                    OperationType.WRITE,
                    device_type_id,
                    phy_addr,
                    0,          # 寄存器地址
                    0x9140      # 强制锁定100Mbps全双工
                )

                print("C22协议1000兆模式设置完成")
                return ErrorCode.SUCCESS.value

            except Exception as e:
                print(f"设置失败: {str(e)}")
                return ErrorCode.UNKNOWN_OPCODE.value



# ================== 测试脚本 ==================
def main():

    mdio = MDIOController()

    # 示例：设备类型 ID 和 PHY 地址
    addr_dev = 1  # 设备类型 ID
    phy_addr = 7  # PHY 地址
    mdio.marvell_88q222x_getMasterSlave(addr_dev, phy_addr)
    
    #mdio_ctrl = MDIOController(port="COM6")
    while True:

    # 执行百兆模式设置
        result =mdio.marvell_88ea1512_setSpeed_fe(
            phy_addr      =2,       # 实际PHY地址
            device_type_id=1  # 设备类型ID
        )
        time.sleep(15.12) 

        result =mdio.marvell_88ea1512_setSpeed_1000M(
            phy_addr      =2,       # 实际PHY地址
            device_type_id=1  # 设备类型ID
        )
        time.sleep(15.12) 



    #mdio.marvell_88q222x_set_speed(ETH_SPEED_100M)
    #mdio.marvell_88q222x_setMasterSlave(0, 1)
    while 0:

        time.sleep(0.12)
        # 测试主从模式
        print("测试主从模式:")
        mdio.marvell_88q222x_getMasterSlave(addr_dev, phy_addr)
        print("Master/Slave 状态:", "Master" if mdio.marvell_88q222x_getMasterSlave(addr_dev, phy_addr) else "Slave")

        # 测试 Auto-Negotiation
        print("测试 Auto-Negotiation:")
        print("Auto-Negotiation 状态:", "启用" if mdio.marvell_88q222x_getAnegEnabled(addr_dev, phy_addr) else "禁用")
        time.sleep(0.02)
        # 测试速度
        print("             测试速度:")
        print("当前速度:", "                                                                                1000Mbps" if mdio.marvell_88q222x_getSpeed(addr_dev, phy_addr) == 1000 else "                           100Mbps")

        # 测试链路状态
        #print("                                                                         测试链路状态: ")
        #print("链路状态:", "连接" if mdio.marvell_88q222x_checkLink(addr_dev, phy_addr) else "                                                 断开")

        # 测试实时链路状态
        print("                                  测试实时链路状态:")
        print("实时链路状态:", "连接" if mdio.marvell_88q222x_getRealTimeLinkStatus(addr_dev, phy_addr) else "                                 断开")

        # 测试 SQI
        print("测试 SQI:")
        sqi = mdio.marvell_88q222x_getSQIReading_8Level(addr_dev, phy_addr)
        print("SQI 值:                                               ", sqi)
    

        #mdio.marvell_88q222x_set_speed(ETH_SPEED_1000M)
        #mdio.marvell_88q222x_setMasterSlave(0, 0)

        time.sleep(0.22)
        # 测试主从模式
        print("测试主从模式:")
        mdio.marvell_88q222x_getMasterSlave(addr_dev, phy_addr)
        print("Master/Slave 状态:", "Master" if mdio.marvell_88q222x_getMasterSlave(addr_dev, phy_addr) else "Slave")

        # 测试 Auto-Negotiation
        print("测试 Auto-Negotiation:")
        print("Auto-Negotiation 状态:", "启用" if mdio.marvell_88q222x_getAnegEnabled(addr_dev, phy_addr) else "禁用")
        time.sleep(0.02)
        # 测试速度
        print("             测试速度:")
        print("当前速度:", "                                                                 1000Mbps" if mdio.marvell_88q222x_getSpeed(addr_dev, phy_addr) == 1000 else "                           100Mbps")

        # 测试链路状态
        #print("                                                                         测试链路状态: ")
        #print("链路状态:", "连接" if mdio.marvell_88q222x_checkLink(addr_dev, phy_addr) else "                                                 断开")

        # 测试实时链路状态
        print("                                  测试实时链路状态:")
        print("实时链路状态:", "连接" if mdio.marvell_88q222x_getRealTimeLinkStatus(addr_dev, phy_addr) else "                                 断开")

        # 测试 SQI
        print("测试 SQI:")
        sqi = mdio.marvell_88q222x_getSQIReading_8Level(addr_dev, phy_addr)
        print("SQI 值:                                               ", sqi)






if __name__ == "__main__":
    main()