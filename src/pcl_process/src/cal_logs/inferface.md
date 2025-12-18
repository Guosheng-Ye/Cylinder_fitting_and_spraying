# 接口参数说明
| 字段名 | 类型 | 说明 |
|--------|------|------|
| distance | float64 | 轨迹点与圆弧表面的距离 |
| step_width | float64 | 工作区域单次喷幅(mm) |
| step_height | float64 | 工作区域单次高度(mm) |
| height | int16 | 工作区总高度，与单次高度成整数倍关系 |
| lap | float64 | 两次喷幅搭接比例（0~1之间） |
| arm_reach | int16 | 机械臂臂展(mm) |
| mode | int | 1手动 2自动 |
| width | int16 | 手动模式下直接透传即可；自动模式需要动态计算后传值 |