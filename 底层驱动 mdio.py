class MarvellPhy:
    # 寄存器地址定义
    MRVL_ORGANIZATION_ID_REG = 0x0034
    MRVL_ORGANIZATION_ID = 0x0001
    MRVL_Q222X_TC10_CONTROL_DEVICE = 0x8034
    MRVL_Q222X_TC10_STATUS_REG_1000 = 0x0012
    MRVL_Q222X_TC10_STATUS_REG_100 = 0x0013
    
    ETH_SPEED_1000M = 0x0001
    ETH_SPEED_100M = 0x0000
    
    MRVL_APHY_OP_MASTER = 0x4000
    MRVL_APHY_OP_SLAVE = 0xBFFF

    def __init__(self, mdio_controller):
        self.mdio = mdio_controller

    def marvell_88q222x_soft_reset(self):
        """执行软复位操作"""
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0x0900,
            val_in=0x8000
        )
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0xFFE4,
            val_in=0x000C
        )

    def apply_b0_pre_init(self):
        """B0版本预初始化"""
        # 使能TX DAC
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0x8033,
            val_in=0x6801
        )
        
        # 禁用自动协商
        self.mdio.set_mdio_reg_whole(
            addr_dev=7,
            phy_addr=0x0200,
            val_in=0x0000
        )

        # 电源管理配置
        self.mdio.set_mdio_reg_whole(
            addr_dev=1,
            phy_addr=0x0000,
            val_in=0x840
        )
        
        # 退出待机状态
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0xFE1B,
            val_in=0x48
        )

        # 设置电源管理断点
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0xFFE4,
            val_in=0x6B6
        )

        # 等待进入电源管理状态
        self._wait_for_power_management()

        # 关闭CM钳位
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0xFE79,
            val_in=0x0000
        )

        # 设置MDI VCM参数
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0xFE07,
            val_in=0x125A
        )
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0xFE09,
            val_in=0x1288
        )
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0xFE08,
            val_in=0x2588
        )
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0xFE11,
            val_in=0x1105
        )

        # 设置辅助放大参数
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0xFE72,
            val_in=0x042C
        )

    def _wait_for_power_management(self):
        """等待电源管理状态就绪"""
        max_retries = 5
        for _ in range(max_retries):
            reg_val = self.mdio.get_mdio_reg_whole(
                addr_dev=3,
                phy_addr=0xFFE4
            )
            if reg_val == 0x06BA:
                return
            # 精确延时100us
            self._precise_delay(100)

    def _precise_delay(self, microseconds):
        """精确延时实现"""
        start = time.perf_counter()
        while (time.perf_counter() - start) * 1_000_000 < microseconds:
            pass

    def apply_fe_init(self, need_prepare: bool):
        """FE/100M初始化"""
        rev_num = self.mdio.get_mdio_reg_whole(
            addr_dev=1,
            phy_addr=self.MRVL_ORGANIZATION_ID_REG,
            reg_num=0x000F
        )

        if rev_num == self.MRVL_Q222X_B0:
            if need_prepare:
                self.apply_b0_pre_init()
            
            # 更新FFE系数
            self.mdio.set_mdio_reg_whole(
                addr_dev=3,
                phy_addr=0xFBBA,
                val_in=0x0CB2
            )
            self.mdio.set_mdio_reg_whole(
                addr_dev=3,
                phy_addr=0xFBBB,
                val_in=0x0C4A
            )

            # 启用CM钳位
            self.mdio.set_mdio_reg_whole(
                addr_dev=3,
                phy_addr=0xFE79,
                val_in=0x0004
            )

    def soft_reset_fe(self):
        """FE软复位"""
        rev_num = self.mdio.get_mdio_reg_whole(
            addr_dev=1,
            phy_addr=self.MRVL_ORGANIZATION_ID_REG,
            reg_num=0x000F
        )

        if rev_num == self.MRVL_Q222X_B0:
            # 执行B0复位序列
            self.mdio.set_mdio_reg_whole(
                addr_dev=3,
                phy_addr=0x0900,
                val_in=0x8000
            )
            self.mdio.set_mdio_reg_whole(
                addr_dev=3,
                phy_addr=0xFFE4,
                val_in=0x000C
            )

    def marvell_88q222x_init_fe2(self, force_master: int):
        """FE2初始化主函数"""
        # 禁用自动协商
        self.mdio.set_mdio_reg_whole(
            addr_dev=7,
            phy_addr=0x0200,
            val_in=0x0000
        )

        # 基础配置
        self.mdio.set_mdio_reg_whole(
            addr_dev=1,
            phy_addr=0x0000,
            val_in=0x840
        )
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0xFE1B,
            val_in=0x48
        )
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0xFFE4,
            val_in=0x6B6
        )

        # LED配置
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0x8000,
            val_in=0xFFF7
        )
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0x8310,
            val_in=0x0030
        )
        self.mdio.set_mdio_reg_whole(
            addr_dev=3,
            phy_addr=0x8016,
            val_in=0x0030
        )

        # 主从模式配置
        force_master = 0x4000 if force_master else 0x0000
        self.mdio.set_mdio_reg_whole(
            addr_dev=1,
            phy_addr=0x0834,
            val_in=force_master
        )

        # 执行复位
        self.marvell_88q222x_soft_reset()

    def apply_ge_init(self, need_prepare: bool):
        """GE/1000M初始化"""
        rev_num = self.mdio.get_mdio_reg_whole(
            addr_dev=1,
            phy_addr=self.MRVL_ORGANIZATION_ID_REG,
            reg_num=0x000F
        )

        if rev_num == self.MRVL_Q222X_B0:
            if need_prepare:
                self.apply_b0_pre_init()
            
            # 速率检测阈值配置
            self.mdio.set_mdio_reg_whole(
                addr_dev=7,
                phy_addr=0x8032,
                val_in=0x2020
            )
            self.mdio.set_mdio_reg_whole(
                addr_dev=7,
                phy_addr=0x8031,
                val_in=0xA28
            )
            self.mdio.set_mdio_reg_whole(
                addr_dev=7,
                phy_addr=0x8031,
                val_in=0xC28
            )

            # 禁用DCL校准
            self.mdio.set_mdio_reg_whole(
                addr_dev=3,
                phy_addr=0xFFDB,
                val_in=0xFC10
            )

            # 禁用DCL复位
            self.mdio.set_mdio_reg_whole(
                addr_dev=3,
                phy_addr=0xFE1B,
                val_in=0x58
            )

            # 启用CM钳位
            self.mdio.set_mdio_reg_whole(
                addr_dev=3,
                phy_addr=0xFE79,
                val_in=0x0004
            )




















            