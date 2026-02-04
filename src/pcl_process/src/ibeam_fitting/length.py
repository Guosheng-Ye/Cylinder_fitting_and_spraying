def calculate_beam_spraying_limit(self, R_max, D_base, D_work, height, base_z_offset):
        """
        R_max: 机械臂最大半径 (mm)
        D_base: 底座中心到腹板表面的水平距离 (mm)1100
        D_work: 喷枪物理长度 + 喷涂挂距 (mm)650
        height: 工字钢腹板高度 (mm)
        base_z_offset: 机械臂底座中心与腹板中心高度线的高度差 (mm)460
                       (底座比腹板中心高则为正)
        """
        # 1. 姿态折减后的有效半径
        R_eff = R_max * 0.75

        # 2. 机械臂法兰中心距离底座中心的水平深度
        D_target = D_base - D_work

        # 3. 计算垂直方向的最极端偏距 (考虑高度差)
        h_top = (height / 2.0) - base_z_offset
        h_bottom = -(height / 2.0) - base_z_offset
        h_max_dist = max(abs(h_top), abs(h_bottom))

        # 4. 几何可行性检查
        if abs(D_target) > R_eff:
            return 0, f"错误：安装距离 D_base ({D_base}) 减去工作间距后超出机械臂半径"

        square_val = R_eff ** 2 - D_target ** 2 - h_max_dist ** 2

        if square_val < 0:
            return 0, "错误：在当前高度偏差或安装距离下，机械臂无法够到腹板边缘角点"

        # 5. 计算结果
        l_half = math.sqrt(square_val)
        total_length = l_half * 2

        return total_length, "正常"