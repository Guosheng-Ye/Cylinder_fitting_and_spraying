# 传入参数
- 切片高度 (当前800mm)
- 机械臂臂展（当前1500mm、由型号决定）
- 搭接比例（0~100%）
- 喷副（固定300mm）
- 手动/自动作业标记（手动：传入倍数）

# 计算参数
- 圆柱半径
- 底座距离（当前1100mm）
- 表面距离（当前700-800mm）

# 输出
- 切片宽（最佳）

## 代办-主控
- 切片总面积（单个*个数）

## 12-16更新
- total_angle计算
### 输入参数
1. 机械臂臂展（当前1500mm）
2. 切片作业高度（当前800mm）
3. **喷副（300mm）**
4. **搭接比例（1~100%）**
5. 末端喷涂工作距离（600-800mm）
6. 工作模式标记
    - 自动模式
    - 手动模式（传入参数倍数N）

### 输出参数
- W$_0$：喷副 * (1-搭接比例)的最优值，需要覆盖完整工作面的取整
- W：**切片宽**，n * W$_0$
```python 
def calculate_angle_in_robot_frame_corrected(
            self,
            robot_max_reach,  # R_arm: 机械臂最大臂长 (球半径)1500
            cylinder_center,  # P_cyl: 圆柱轴线上一点 [x, y, z] (机械臂坐标系)
            cylinder_axis,  # V_axis: 圆柱轴线方向 [i, j, k]
            cylinder_radius,  # r_cyl: 圆柱半径
            offset_dist,  # D_offset: 喷涂距离
            regional_height  # H: 总工作高度
    ):
```
