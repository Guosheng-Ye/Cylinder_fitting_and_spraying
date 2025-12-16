import numpy as np
def calculate_angle_in_robot_frame_corrected(
            robot_max_reach,  # R_arm: 机械臂最大臂长 (球半径)1500
            cylinder_center,  # P_cyl: 圆柱轴线上一点 [x, y, z] (机械臂坐标系)
            cylinder_axis,  # V_axis: 圆柱轴线方向 [i, j, k]
            cylinder_radius,  # r_cyl: 圆柱半径
            offset_dist,  # D_offset: 喷涂距离
            regional_height  # H: 总工作高度
    ):
        # 1. --- 确定最远工作高度 Z_height ---
        Z_base = 0.0
        Z_cyl_center = cylinder_center[2]  # 圆柱轴心在机械臂坐标系下的 Z 值
        H = regional_height

        # 计算工作范围的上下限
        Z_max_limit = Z_cyl_center + H / 2
        Z_min_limit = Z_cyl_center - H / 2

        # 确定工作范围内距离 Z=0 最远的距离 (最大垂直距离)
        Delta_Z_max = max(abs(Z_max_limit), abs(Z_min_limit))

        print(f"工作范围 Z: [{Z_min_limit:.2f}, {Z_max_limit:.2f}]")
        print(f"距离底座最远垂直距离 Delta_Z_max: {Delta_Z_max:.2f}")

        # 2. --- 计算垂直距离 D (保持不变) ---
        P_base = np.array([0.0, 0.0, 0.0])
        P_cyl = np.array(cylinder_center)
        V_axis = np.array(cylinder_axis)
        V_axis = V_axis / np.linalg.norm(V_axis)
        w = P_base - P_cyl
        D = np.linalg.norm(np.cross(w, V_axis))
        print(f"底座到圆柱轴线的垂直距离 D: {D:.2f}")

        # 3. --- 计算有效臂长 R_eff ---
        R_task = cylinder_radius + offset_dist

        if Delta_Z_max >= robot_max_reach:
            return 0, "Error: 工作高度超出机械臂最大范围"

        # R_eff = 修正后的水平有效臂长
        R_eff = np.sqrt(robot_max_reach ** 2 - Delta_Z_max ** 2)
        print(f"修正后的有效水平臂长 R_eff: {R_eff:.2f}")

        # 4. --- 余弦定理计算角度 (保持不变) ---
        if R_eff < abs(D - R_task):
            return 0, "Error: 无法触及表面"

        # R_task = b, D = c, R_eff = a
        cos_alpha = (R_task ** 2 + D ** 2 - R_eff ** 2) / (2 * R_task * D)

        cos_alpha = np.clip(cos_alpha, -1.0, 1.0)

        alpha = np.arccos(cos_alpha)
        total_angle = 2 * np.degrees(alpha)

        return total_angle, "计算成功"

def main():
    # TEST  
    robot_reach = 1400
    angle, msg = calculate_angle_in_robot_frame_corrected(
        robot_max_reach=robot_reach,
        cylinder_center=[-996.669, -1112.73, 480.369],
        cylinder_axis=[-0.00514, -0.02479, -0.999679],
        cylinder_radius=351,
        offset_dist=700,
        regional_height=800  # 修正后的总工作高度 H
    )

    print("-" * 30)
    print(f"计算结果: {msg}")
    print(f"圆柱表面可覆盖的总角度: {angle:.2f}°")


if __name__ == "__main__":
    main()

# /cylinder_fitting/cal.py
# 工作范围 Z: [80.37, 880.37]
# 距离底座最远垂直距离 Delta_Z_max: 880.37
# 底座到圆柱轴线的垂直距离 D: 1504.00
# 修正后的有效水平臂长 R_eff: 1088.55
# ------------------------------
# 计算结果: 计算成功
# 圆柱表面可覆盖的总角度: 92.73°