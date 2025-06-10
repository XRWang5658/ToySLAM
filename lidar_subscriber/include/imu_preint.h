/*
Preintegration class for IMU Data,
Used for learning IMU preintegration algorithms,
Following VINS-MONO approach

Written by Xiangru Wang
Date: 2025-04-29

*/

#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <vector>
#include "utility.h"

enum StateOrder {
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12,
};

class imu_preint
{
public:
    // constructor
    imu_preint(){
        // initialize the parameters
        ba.setZero();
        bg.setZero();

        alpha.setZero();
        beta.setZero();
        gamma.setIdentity();

        alpha_new.setZero();
        beta_new.setZero();
        gamma_new.setIdentity();

        stamp_buf.clear();
        acc_buf.clear();
        gyro_buf.clear();

        sum_dt = 0.0;
        jacobian.setIdentity();
        covariance.setIdentity();
        covariance = 1e-8 * covariance;
        covariance.setZero();

        // set default gravity  
        set_gravity(9.785);
        // set default noise
        set_noise(0.01, 0.01, 0.01, 0.01);
        // set default bias
        setBias(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }

    bool set_noise(const double ACC_N, const double GYR_N, const double ACC_W, const double GYR_W){
        acc_noise_sigma_ = ACC_N;
        gyro_noise_sigma_ = GYR_N;
        acc_bias_walk_sigma_continuous_ = ACC_W;
        gyro_bias_walk_sigma_continuous_ = GYR_W;

        noise = Eigen::Matrix<double, 18, 18>::Zero();
        noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
    }

    bool push_back(double stamp, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro)
    {
        // store the IMU data
        stamp_buf.push_back(stamp);
        acc_buf.push_back(acc);
        gyro_buf.push_back(gyro);

        // ROS_INFO("IMU data pushed back: %f", stamp);

        // if this is the first IMU data, return directly to avoid integration
        if (stamp_buf.size() <= 1)
        {
            // ROS_WARN("First IMU data, no integration needed");
            return true;
        }
        // else, perform the integration in real time

        if(!propagate(stamp_buf.size() - 1))
        {   
            ROS_WARN("IMU data propagation failed");
            return false;
        }

        return true;
    }

    bool setBias(const Eigen::Vector3d &bias_acc, const Eigen::Vector3d &bias_gyro)
    {
        ba = bias_acc;
        bg = bias_gyro;
        return true;
    }

    bool set_gravity(double gravity_magnitude){
        g_world = Eigen::Vector3d(0, 0, gravity_magnitude);
        return true;
    }

    bool midpoint_integrate(double dt,
                            const Eigen::Vector3d &raw_acc1, const Eigen::Vector3d &raw_acc2,
                            const Eigen::Vector3d &raw_gyro1, const Eigen::Vector3d &raw_gyro2,
                            const Eigen::Vector3d &_alpha, const Eigen::Vector3d &_beta, const Eigen::Quaterniond &_gamma,
                            Eigen ::Vector3d &alpha_new, Eigen::Vector3d &beta_new, Eigen::Quaterniond &gamma_new, bool update_jacobian = true){
        // calculate the midpoint gamma first
        Eigen::Vector3d gyro_avg = 0.5 * (raw_gyro1 + raw_gyro2) - bg;

        // calculate the new gamma
        gamma_new = _gamma * Eigen::Quaterniond(1, 0.5* gyro_avg(0) * dt, 0.5* gyro_avg(1) * dt, 0.5* gyro_avg(2) * dt);
        gamma_new.normalize();

        // ROS_INFO("Midpoint gamma: %f %f %f %f", gamma_new.w(), gamma_new.x(), gamma_new.y(), gamma_new.z());

        //translate the acceleration to the world frame
        Eigen::Vector3d acc1 = _gamma * (raw_acc1 - ba);
        Eigen::Vector3d acc2 = gamma_new * (raw_acc2 - ba);
        Eigen::Vector3d acc_avg = 0.5 * (acc1 + acc2);

        // calculate the new beta
        beta_new = _beta + acc_avg * dt;

        // calculate the new alpha
        alpha_new = _alpha + _beta * dt + 0.5 * acc_avg * dt * dt;     
        
        // update the jacobian and covariance if requested
        if(update_jacobian){
            Eigen::Vector3d w_x_  = gyro_avg;
            Eigen::Vector3d a0x = raw_acc1 - ba;
            Eigen::Vector3d a1x = raw_acc2 - ba;
            Eigen::Matrix3d R_w_x, R_a0x, R_a1x;

            R_w_x << 0, -w_x_(2), w_x_(1),
                     w_x_(2), 0, - w_x_(0),
                     -w_x_(1), w_x_(0), 0;

            R_a0x << 0, -a0x(2), a0x(1),
                     a0x(2), 0, - a0x(0),
                     -a0x(1), a0x(0), 0;

            R_a1x << 0, -a1x(2), a1x(1),
                     a1x(2), 0, - a1x(0),
                     -a1x(1), a1x(0), 0;

            Eigen::MatrixXd F = Eigen::Matrix<double, 15, 15>::Zero();
            F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * _gamma.toRotationMatrix() * R_a0x * dt * dt + 
                                  -0.25 * gamma_new.toRotationMatrix() * R_a1x * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt * dt;
            F.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
            F.block<3, 3>(0, 9) = -0.25 * (_gamma.toRotationMatrix() + gamma_new.toRotationMatrix()) * dt * dt;
            F.block<3, 3>(0, 12)= -0.25 * gamma_new.toRotationMatrix() * R_a1x * dt * dt * - dt;

            F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - R_w_x * dt;
            F.block<3, 3>(3, 12)= -1.0 * Eigen::Matrix3d::Identity() * dt;

            F.block<3, 3>(6, 3) = -0.5 * _gamma.toRotationMatrix() * R_a0x * dt + 
                                  -0.5 * gamma_new.toRotationMatrix() * R_a1x * (Eigen::Matrix3d::Identity() - R_w_x * dt) * dt;
            F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (_gamma.toRotationMatrix() + gamma_new.toRotationMatrix()) * dt;
            F.block<3, 3>(6, 12)= -0.5 * gamma_new.toRotationMatrix() * R_a1x * dt * - dt;

            F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(12,12)= Eigen::Matrix3d::Identity();

            Eigen::MatrixXd V = Eigen::Matrix<double, 15, 18>::Zero();
            V.block<3, 3>(0, 0) = 0.25 * _gamma.toRotationMatrix() * dt * dt;
            V.block<3, 3>(0, 3) = - 0.25 * gamma_new.toRotationMatrix() * R_a1x * dt * dt * 0.5 * dt;
            V.block<3, 3>(0, 6) = 0.25 * gamma_new.toRotationMatrix() * dt * dt;
            V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);

            V.block<3, 3>(3, 3) = 0.5 * Eigen::Matrix3d::Identity() * dt;
            V.block<3, 3>(3, 9) = 0.5 * Eigen::Matrix3d::Identity() * dt;

            V.block<3, 3>(6, 0) = 0.5 * _gamma.toRotationMatrix() * dt;
            V.block<3, 3>(6, 3) = - 0.5 * gamma_new.toRotationMatrix() * R_a1x * dt * 0.5 * dt;
            V.block<3, 3>(6, 6) = 0.5 * gamma_new.toRotationMatrix() * dt;
            V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);

            V.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity() * dt;
            V.block<3, 3>(12, 15)= Eigen::Matrix3d::Identity() * dt;

            jacobian  = F * jacobian;
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
        }

        return true;
    }

    bool propagate(int index){
        // get the imu data at the given index
        double dt = stamp_buf[index] - stamp_buf[index - 1];
        if (dt <= 0.0)
        {
            ROS_INFO("imu preintegration error! dt <= 0.0");
            return false;
        }

        if(dt < 1e-9)
        {
            ROS_WARN("IMU preintegration error! dt is too small");
            return false;
        }

        Eigen::Vector3d raw_acc1 = acc_buf[index - 1];
        Eigen::Vector3d raw_acc2 = acc_buf[index];
        Eigen::Vector3d raw_gyro1 = gyro_buf[index - 1];
        Eigen::Vector3d raw_gyro2 = gyro_buf[index];

        Eigen::Vector3d alpha_propagated, beta_propagated;
        Eigen::Quaterniond gamma_propagated;

        if(!midpoint_integrate(dt, raw_acc1, raw_acc2, raw_gyro1, raw_gyro2, alpha, beta, gamma, alpha_propagated, beta_propagated, gamma_propagated, true))
        {   
            ROS_WARN("IMU preintegration error! midpoint integration failed");
            return false;
        }

        // update the alpha, beta, gamma
        alpha = alpha_propagated;
        beta = beta_propagated;
        gamma = gamma_propagated;

        
        sum_dt += dt;

        return true;
    }

    bool repropagate(const Eigen::Vector3d &ba_new, const Eigen::Vector3d &bg_new)
    {
        // reset the bias
        setBias(ba_new, bg_new);

        // reset the variables
        alpha.setZero();
        beta.setZero();
        gamma.setIdentity();
        alpha_new.setZero();
        beta_new.setZero();
        gamma_new.setIdentity();
        jacobian.setIdentity();
        covariance.setZero();
        sum_dt = 0.0;

        // repropagate the IMU data
        for (size_t i = 1; i < stamp_buf.size(); ++i)
        {
            if(!propagate(i))
            {
                return false;
            }
        }
        return true;
    }

    Eigen::Matrix<double, 15, 1> evaluate(
        const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
        const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj
    ){
        Eigen::Matrix<double, 15, 1> residuals;

        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

        Eigen::Vector3d dba = Bai - ba;
        Eigen::Vector3d dbg = Bgi - bg;

        Eigen::Quaterniond corrected_delta_q = gamma * Utility::deltaQ(dq_dbg * dbg);
        corrected_delta_q.normalize();
        Eigen::Vector3d corrected_delta_v = beta + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = alpha + dp_dba * dba + dp_dbg * dbg;

        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * g_world * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (g_world * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
        return residuals;
    }

    bool reset()
    {

        alpha.setZero();
        beta.setZero();
        gamma.setIdentity();

        alpha_new.setZero();
        beta_new.setZero();
        gamma_new.setIdentity();

        stamp_buf.clear();
        acc_buf.clear();
        gyro_buf.clear();

        jacobian.setIdentity();
        covariance.setIdentity();
        covariance = 1e-8 * covariance;
        covariance.setZero();

        sum_dt = 0.0;

        return true;
    }

    Eigen::MatrixXd get_jacobian() const
    {
        return jacobian;
    }

    double get_sum_dt() const
    {
        return sum_dt;
    }

    double getSumDt() const { return sum_dt; }

    Eigen::Matrix3d getJacobianDpDba() const { return jacobian.block<3, 3>(O_P, O_BA); }
    Eigen::Matrix3d getJacobianDpDbg() const { return jacobian.block<3, 3>(O_P, O_BG); }
    Eigen::Matrix3d getJacobianDqDbg() const { return jacobian.block<3, 3>(O_R, O_BG); }
    Eigen::Matrix3d getJacobianDvDba() const { return jacobian.block<3, 3>(O_V, O_BA); }
    Eigen::Matrix3d getJacobianDvDbg() const { return jacobian.block<3, 3>(O_V, O_BG); }
    Eigen::Vector3d getGravity() const { return g_world; } // Also needed

    // Consider adding getters for corrected measurements if needed outside
    // Eigen::Vector3d getCorrectedDeltaP(const Eigen::Vector3d& Bai, const Eigen::Vector3d& Bgi) const { ... }
    // Eigen::Vector3d getCorrectedDeltaV(const Eigen::Vector3d& Bai, const Eigen::Vector3d& Bgi) const { ... }
    // Eigen::Quaterniond getCorrectedDeltaQ(const Eigen::Vector3d& Bgi) const { ... }

    // Also, consider making the results of evaluate (alpha, beta, gamma, corrected versions) accessible if needed elsewhere
    Eigen::Vector3d getDeltaAlpha() const { return alpha; }
    Eigen::Vector3d getDeltaBeta() const { return beta; }
    Eigen::Quaterniond getDeltaGamma() const { return gamma; }

    Eigen::Quaterniond getDeltaQ() const { return gamma; }

    Eigen::Vector3d getBg() const { return bg; }

    Eigen::Vector3d getBa() const { return ba; }

    Eigen::MatrixXd getCovariance() const { return covariance; }

    Eigen::MatrixXd getJacobian() const { return jacobian; }

    double getAccNoiseSigma() const {
        return acc_noise_sigma_;
   }
    double getGyroNoiseSigma() const {
        return gyro_noise_sigma_;
    }
    double getAccBiasWalkSigma() const {
        return acc_bias_walk_sigma_continuous_; 
    }
    double getGyroBiasWalkSigma() const {
        return gyro_bias_walk_sigma_continuous_; // Assuming GYR_W stored as member
    }

private:

    double sum_dt;

    // basic data storage
    std::vector<double> stamp_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyro_buf;

    // bias term
    Eigen::Vector3d ba, bg;

    // gravity vector
    Eigen::Vector3d g_world;

    // integration result
    Eigen::Vector3d alpha; // delta_p
    Eigen::Vector3d beta; // delta_v
    Eigen::Quaterniond gamma; // delta_q

    // temp variables for computed values in each integration step
    Eigen::Vector3d alpha_new, beta_new;
    Eigen::Quaterniond gamma_new;

    Eigen::Matrix<double, 15, 15> jacobian, covariance;
    Eigen::Matrix<double, 18, 18> noise;

    double acc_noise_sigma_ = 0.0;
    double gyro_noise_sigma_ = 0.0;
    double acc_bias_walk_sigma_continuous_ = 0.0; // Store the continuous time random walk sigma
    double gyro_bias_walk_sigma_continuous_ = 0.0;// Store the continuous time random walk sigma

};
