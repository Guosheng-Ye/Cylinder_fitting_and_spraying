#include "Eigen/Dense"
#include "ros/ros.h"

// 曲臂工作位姿参考数据 (全部单位: 毫米 mm)
// 【位姿1 - 转台角度正向90°（逆时针）】
//  末端A位置坐标：X = 590mm, Y = 2470mm, Z = 2560mm

//【位姿2 - 转台角度反向90°（顺时针）】
// 位置坐标：X = -0.46米, Y = -2.50米, Z = 2.58米
// 未使用

// 曲臂位姿与雷达坐标转换类
// 定义
// -    基座O(X前, Y左, Z上)
// -    曲臂末端A
// -    雷达B(Y向前,X向右,Z向上)
// -    小机械臂C(X前,Y左,Z上)
// 假设: 基座O坐标系 X前(车头), Y左, Z上. 转台+90°(逆时针)后, 曲臂末端A的局部X轴指向基座Y正方向.
// 已知: A在O中的位置(590, 2470, 2560)mm. 雷达B相对A偏移: X前+820mm, Y≈0mm, Z下-160mm.
//      C相对于曲臂末端A的偏移：X前630mm,Y≈0mm,Z上160mm.
// 雷达姿态假设与A一致(纯平面旋转+90°). 点(p_x,p_y,p_z)输入为雷达B坐标系下的点(单位mm), 输出为基座O坐标系下坐标(单位mm).
// class FlexedArmTF {
// public:
//     FlexedArmTF() {
//         // A 在 O 中平移 (mm)
//         t_OA_ = Eigen::Vector3d(590.0, 2470.0, 2560.0);

//         // A->B 安装偏移 (在 A 坐标系下, mm)
//         t_AB_ = Eigen::Vector3d(820.0, 0.0, -160.0);

//         // O->A 旋转(+90°)
//         double yaw = M_PI / 2.0;
//         R_OA_ << std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw), std::cos(yaw), 0, 0, 0, 1;

//         // A->B 旋转设为 R_OA^T，使得 R_OB = R_OA * R_AB = I
//         R_AB_ = R_OA_.transpose();

//         // 雷达在 O 中位置: t_OB = t_OA + R_OA * t_AB
//         t_OB_ = t_OA_ + R_OA_ * t_AB_;
//     }

//     // 输入: 雷达坐标系点 p_B (mm)，输出: 基座 O 坐标系点 (mm)
//     Eigen::Vector3d transformPose1(const Eigen::Vector3d &p_B) const {
//         // R_OB = I
//         return t_OB_ + p_B;
//     }

//     Eigen::Vector3d lidarInBase() const { return t_OB_; }

// private:
//     Eigen::Vector3d t_OA_; // A在O中平移 (mm)
//     Eigen::Vector3d t_AB_; // B相对A平移 (mm)
//     Eigen::Vector3d t_OB_; // 预计算B在O中平移 (mm)
//     Eigen::Matrix3d R_OA_; // O->A旋转
//     Eigen::Matrix3d R_AB_; // A->B旋转
// };

// 说明: 全部单位 mm
// O: 基座  (X前 Y左 Z上)
// A: 曲臂末端 (转台旋转 +90° 后其自身坐标被旋转)
// B: 雷达 (侧装, 通过设置 R_AB = R_OA^T 使最终与 O 对齐)
// C: 小机械臂基座 (需求: 与 O 坐标轴对齐, 仅平移偏移)
// 已知偏移(在 A 坐标系表达):
//   A->B: (820, 0, -160)
//   A->C: (630, 0, +160)

/// @brief 说明: 全部单位 mm
/// O: 基座  (X前 Y左 Z上)
/// A: 曲臂末端 (转台旋转 +90° 后其自身坐标被旋转)
/// B: 雷达 (侧装, 通过设置 R_AB = R_OA^T 使最终与 O 对齐)
/// C: 小机械臂基座 (需求: 与 O 坐标轴对齐, 仅平移偏移)
///   已知偏移(在 A 坐标系表达):
///   A->B: (820, 0, -160)
///   A->C: (630, 0, +160)
class FlexedArmTF {
public:
    FlexedArmTF() {
        // A 在 O 中平移
        t_OA_ = Eigen::Vector3d(590.0, 2470.0, 2560.0);

        // 偏移 (A 坐标系)
        t_AB_ = Eigen::Vector3d(820.0, 0.0, -160.0);
        t_AC_ = Eigen::Vector3d(630.0, 0.0, 160.0);

        // O->A 旋转(+90°)
        double yaw = M_PI / 2.0;
        R_OA_ << std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw), std::cos(yaw), 0, 0, 0, 1;

        // 令 B 与 O 对齐: R_AB = R_OA^T  → R_OB = I
        R_AB_ = R_OA_.transpose();
        // 令 C 与 O 对齐: R_AC = R_OA^T  → R_OC = I
        R_AC_ = R_OA_.transpose();

        // 位置: t_OB = t_OA + R_OA * t_AB
        t_OB_ = t_OA_ + R_OA_ * t_AB_;
        // 位置: t_OC = t_OA + R_OA * t_AC
        t_OC_ = t_OA_ + R_OA_ * t_AC_;

        // 由于 R_OB = R_OC = I, 各自与 O 坐标轴一致
    }

    // 雷达 B 坐标 -> 基座 O 坐标
    Eigen::Vector3d to_base_from_lidar(const Eigen::Vector3d &p_B) const { return t_OB_ + p_B; }

    // 雷达 B 坐标 -> 小机械臂 C 坐标
    // 推导: p_O = t_OB_ + p_B;  p_O = t_OC_ + p_C  => p_C = p_B + (t_OB_ - t_OC_)
    Eigen::Vector3d to_robot_from_lidar(const Eigen::Vector3d &p_B) const { return p_B + (t_OB_ - t_OC_); }
    // 基座 O 坐标 -> 小机械臂 C 坐标
    Eigen::Vector3d base_to_c(const Eigen::Vector3d &p_O) const { return p_O - t_OC_; }

    // 小机械臂 C 坐标 -> 基座 O 坐标
    Eigen::Vector3d c_to_base(const Eigen::Vector3d &p_C) const { return t_OC_ + p_C; }

    Eigen::Vector3d lidar_in_base() const { return t_OB_; }
    Eigen::Vector3d arm_c_in_base() const { return t_OC_; }

private:
    // 平移
    Eigen::Vector3d t_OA_;
    Eigen::Vector3d t_AB_;
    Eigen::Vector3d t_AC_;
    Eigen::Vector3d t_OB_;
    Eigen::Vector3d t_OC_;

    // 旋转
    Eigen::Matrix3d R_OA_;
    Eigen::Matrix3d R_AB_;
    Eigen::Matrix3d R_AC_;
};