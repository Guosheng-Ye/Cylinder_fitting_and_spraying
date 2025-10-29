# 批量目标点列表（去除 batch_id 版本）
# 一个消息包含所有要执行的点位；下游按 id 升序执行。
# 若需再执行同一组点位，通过导航状态重置 ( /achieve_goal false->true ) 清空行为树内部缓存。

TargetPose[] targets
