# 11. 雷达点云预处理

`RSLidarFilterNode` 订阅 `PointXYZI` 点云，遍历所有点；它会跳过任一坐标为 NaN 的点，但没有用 `std::isfinite()` 显式过滤正负无穷值。随后生成两份输出：

```cpp
angle = atan2(point.x, point.y)
point.y > 0
```

| 输出 | 实际半角 | 总扇区 | 用途 |
|---|---:|---:|---|
| `/rslidar_filtered_cylinder` | 40° | 80° | 圆柱候选区域 |
| `/rslidar_filtered_ibeam` | 50° | 100° | 工字钢候选区域 |

过滤节点不执行距离、高度、体素或异常点过滤；这些由下游拟合器处理。输出保留输入 header 和点强度字段。watchdog 每 1 秒检查一次，超过 2 秒未收到数据时告警，恢复时记录恢复日志。

源码中的个别注释和 launch 说明仍写 ±20°/±45°，以头文件常量 `40.0`/`50.0` 和实际判断为准。

---

[← 返回 README](../README.md)
