#include "imu_preint.h" // 您的预积分头文件
#include "utility.h"    // 您的工具类头文件 (需要包含 Utility::deltaQ 等)
#include <iostream>     // 用于 checkJacobian 中的输出
#include <Eigen/Dense> // Eigen 库
#include <Eigen/Geometry> // Eigen 库的几何模块
#include <vector>      // 用于 std::vector
#include <map>         // 用于 std::map
#include <string>      // 用于 std::string
#include <cmath>       // 用于 std::sqrt

int main() {
    // 1. 创建 imu_preint 对象实例
    // 构造函数会设置默认的噪声、重力 (0,0,9.785) 和零偏置
    imu_preint my_integrator;

    // 2. 定义一个特定的IMU积分小时间段 (dt_check) 的起始和结束测量值
    double dt_check = 0.01; // 时间间隔 (s)

    // IMU 测量值 @ k (时间间隔开始)
    // 假设IMU静止，Z轴向上，加速度计测量重力加速度（由于我们的g_world是[0,0,9.785]，所以静止时读数也近似[0,0,9.785]）
    // 并有微小的x,y方向加速度和角速度
    Eigen::Vector3d acc0_check(0.05, -0.02, 9.80); // m/s^2
    Eigen::Vector3d gyro0_check(0.001, -0.002, 0.003); // rad/s

    // IMU 测量值 @ k+1 (时间间隔结束)
    // 假设状态发生微小变化
    Eigen::Vector3d acc1_check(0.06, -0.01, 9.79);   // m/s^2
    Eigen::Vector3d gyro1_check(0.0015, -0.0018, 0.0032); // rad/s

    // 3. 定义这个小时间段开始时的预积分状态 (alpha_k, beta_k, gamma_k)
    // 对于检查由 (acc0, gyro0) 和 (acc1, gyro1) 定义的 *单步* 积分的雅可比矩阵，
    // 我们可以假设这是预积分段的开始，所以初始预积分量为零/单位。
    // alpha, beta 是相对于某个更早的i时刻的预积分量，在第一步时它们是0。
    Eigen::Vector3d alpha_k = Eigen::Vector3d::Zero();          // 初始预积分位置增量 (m)
    Eigen::Vector3d beta_k = Eigen::Vector3d::Zero();           // 初始预积分速度增量 (m/s)
    Eigen::Quaterniond gamma_k = Eigen::Quaterniond::Identity(); // 初始预积分旋转增量

    // 4. 定义用于这个小时间段计算的线性化偏置点 (ba_lin, bg_lin)
    // 这些是用于计算当前积分步骤中F和V矩阵的偏置值。
    // 通常来自前一个优化步骤的估计，或一个初始猜测。
    Eigen::Vector3d ba_lin(0.01, -0.005, 0.02);  // 加速度计偏置 m/s^2
    Eigen::Vector3d bg_lin(0.0005, 0.001, -0.0008); // 陀螺仪偏置 rad/s

    // 5. 调用 checkJacobian
    // checkJacobian 函数内部会使用传递的 current_ba, current_bg (即 ba_lin, bg_lin)
    // 作为该单步计算的偏置线性化点，并临时设置 this->ba 和 this->bg。
    std::cout << "Starting Jacobian Check for imu_preint..." << std::endl;
    my_integrator.checkJacobian(dt_check,
                                acc0_check, gyro0_check,
                                acc1_check, gyro1_check,
                                alpha_k, beta_k, gamma_k,
                                ba_lin, bg_lin);

    // 如果您想测试累积雅可比 (this->jacobian)，您需要先进行一些实际的 push_back 操作：
    // std::cout << "\n--- Example of pushing data first (not directly for checkJacobian single step) ---" << std::endl;
    // imu_preint real_integrator;
    // real_integrator.setBias(ba_lin, bg_lin); // Set bias for integration
    // double current_stamp = 0.0;
    // real_integrator.push_back(current_stamp, acc0_check, gyro0_check); // First point, just stores
    // current_stamp += dt_check;
    // real_integrator.push_back(current_stamp, acc1_check, gyro1_check); // Second point, integrates
    // std::cout << "Cumulative Jacobian after one step:\n" << real_integrator.getJacobian() << std::endl;
    // std::cout << "Cumulative Covariance after one step:\n" << real_integrator.getCovariance() << std::endl;
    // 请注意，checkJacobian 是设计用来检查 *单步* 的 F 和 V，而不是累积的 this->jacobian。

    return 0;
}