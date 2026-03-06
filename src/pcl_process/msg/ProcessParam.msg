float64 distance       # 轨迹点与圆弧表面的距离
float64 step_width     # 工作区域单次喷幅(mm)
float64 step_height    # 工作区域单次高度(mm)
float64 height		   # 工作区总高度，与单次高度成整数倍关系
float64 lap            # 两次喷幅搭接比例（0~1之间）
float64 arm_reach      # 机械臂臂展(mm)
int16   mode           # 1手动 2自动
int16   k              # 手动模式下直接透传即可；自动模式需要动态计算后传值
float64 w0             # 工作区域单次喷幅(mm)