#ifndef GNSS_PARSER_H
#define GNSS_PARSER_H

#include <ros/ros.h>
#include <ros/message_event.h>
#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>
#include <cmath>
#include "gnss_tools.h"

// Include message headers for all supported types
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <novatel_msgs/INSPVAX.h>
#include <nav_msgs/Odometry.h>

namespace gnss_constants {
    constexpr double WGS84_A = 6378137.0;
    constexpr double WGS84_F = 1.0 / 298.257223563;
    constexpr double GPS_UNIX_EPOCH_OFFSET = 315964800.0;
    constexpr double SECONDS_PER_WEEK = 604800.0;
    constexpr double CURRENT_LEAP_SECONDS = 18.0;
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / M_PI;
}

struct GnssMeasurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d position_std_dev = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_std_dev = Eigen::Vector3d::Zero();
    Eigen::Vector3d orientation_std_dev_rpy = Eigen::Vector3d::Zero();
    bool position_valid = false;
    bool velocity_valid = false;
    bool orientation_valid = false;
};

// --- 抽象基类 (现在只包含通用工具) ---
class GnssParser {
public:
    virtual ~GnssParser() = default;
    void setEnuReference(double lat, double lon, double alt) {
        ref_latitude_ = lat; ref_longitude_ = lon; ref_altitude_ = alt;
        has_gps_reference_ = true;
        enu_ref_ << ref_longitude_, ref_latitude_, ref_altitude_;
    }
    bool hasEnuReference() const { return has_gps_reference_; }
    void reset() {
        has_gps_reference_ = false;
        ref_latitude_ = 0.0;
        ref_longitude_ = 0.0;
        ref_altitude_ = 0.0;
        enu_ref_.setZero();
    }
protected:
    bool has_gps_reference_ = false;
    double ref_latitude_ = 0.0, ref_longitude_ = 0.0, ref_altitude_ = 0.0;
    Eigen::Vector3d enu_ref_;
    Eigen::Vector3d convertGpsToEnu(double latitude, double longitude, double altitude) const {
        if (!has_gps_reference_) {
            ROS_WARN_THROTTLE(5.0, "ENU reference not set, cannot convert GPS to ENU. Returning zero vector.");
            return Eigen::Vector3d::Zero();
        }
        
        const double e_sq = gnss_constants::WGS84_F * (2.0 - gnss_constants::WGS84_F);
        double ref_lat_rad = ref_latitude_ * gnss_constants::DEG_TO_RAD;
        double N = gnss_constants::WGS84_A / sqrt(1.0 - e_sq * sin(ref_lat_rad) * sin(ref_lat_rad));

        double e_val = (N + ref_altitude_) * cos(ref_lat_rad) * ((longitude - ref_longitude_) * gnss_constants::DEG_TO_RAD);
        double n_val = (N * (1 - e_sq) + ref_altitude_) * ((latitude - ref_latitude_) * gnss_constants::DEG_TO_RAD);
        double u_val = altitude - ref_altitude_;
        
        return Eigen::Vector3d(e_val, n_val, u_val);
    }

    double gpsToUnixTime(uint32_t week, double secs) const {
        double gps_time = static_cast<double>(week) * gnss_constants::SECONDS_PER_WEEK + secs;
        return gps_time + gnss_constants::GPS_UNIX_EPOCH_OFFSET - gnss_constants::CURRENT_LEAP_SECONDS;
    }
};

// --- 具体解析器 (parse方法参数已修改) ---
class GnssCommParser : public GnssParser {
public:
    std::optional<GnssMeasurement> parse(const gnss_comm::GnssPVTSolnMsg::ConstPtr& msg) {
        if (!msg || !msg->valid_fix || msg->fix_type < 2) return std::nullopt;

        GnssMeasurement meas;
        meas.timestamp = gpsToUnixTime(msg->time.week, msg->time.tow);
        
        if (!this->hasEnuReference()) {
            this->setEnuReference(msg->latitude, msg->longitude, msg->altitude);
        }
        meas.position = convertGpsToEnu(msg->latitude, msg->longitude, msg->altitude);
        meas.position_valid = true;
        meas.position_std_dev << msg->h_acc, msg->h_acc, msg->v_acc;

        meas.velocity = Eigen::Vector3d(msg->vel_e, msg->vel_n, -msg->vel_d);
        meas.velocity_std_dev << msg->vel_acc, msg->vel_acc, msg->vel_acc;
        meas.velocity_valid = true;
        meas.orientation_valid = false;
        return meas;
    }
};

class InspvaxParser : public GnssParser {
public:
    std::optional<GnssMeasurement> parse(const novatel_msgs::INSPVAX::ConstPtr& msg) {
        if (!msg) return std::nullopt;
        
        GnssMeasurement meas;
        meas.timestamp = gpsToUnixTime(msg->header.gps_week, msg->header.gps_week_seconds / 1000.0);

        if (!this->hasEnuReference()) {
            this->setEnuReference(msg->latitude, msg->longitude, msg->altitude);
        }
        meas.position = convertGpsToEnu(msg->latitude, msg->longitude, msg->altitude);
        meas.position_valid = true;
        meas.position_std_dev << msg->longitude_std, msg->latitude_std, msg->altitude_std;

        Eigen::Vector3d body_velocity_rfu(msg->east_velocity, msg->north_velocity, msg->up_velocity);
        meas.velocity = convertRfuToEnu(body_velocity_rfu, msg->roll, msg->pitch, msg->azimuth);
        meas.velocity_valid = true;
        meas.velocity_std_dev << msg->east_velocity_std, msg->north_velocity_std, msg->up_velocity_std;

        // ... (姿态和姿态不确定性的转换)
        meas.orientation_valid = false;
        
        return meas;
    }
private:
        Eigen::Vector3d convertRfuToEnu(const Eigen::Vector3d& v_rfu, double roll_deg, double pitch_deg, double yaw_deg_from_north_clockwise) const {
        double roll_rad = roll_deg * gnss_constants::DEG_TO_RAD;
        double pitch_rad = pitch_deg * gnss_constants::DEG_TO_RAD;
        double yaw_ned_rad = yaw_deg_from_north_clockwise * gnss_constants::DEG_TO_RAD;

        // RFU (Right, Forward, Up) to FRD (Forward, Right, Down)
        Eigen::Vector3d v_frd;
        v_frd(0) = v_rfu(1);  // Forward
        v_frd(1) = v_rfu(0);  // Right
        v_frd(2) = -v_rfu(2); // Down

        // Body(FRD) to NED rotation
        Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw_ned_rad, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d R_frd_to_ned = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
        
        // Velocity in NED
        Eigen::Vector3d v_ned = R_frd_to_ned * v_frd;
        
        // NED (North, East, Down) to ENU (East, North, Up)
        return Eigen::Vector3d(v_ned(1), v_ned(0), -v_ned(2));
    }
};

class OdometryParser : public GnssParser {
public:
    std::optional<GnssMeasurement> parse(const nav_msgs::Odometry::ConstPtr& msg) {
        GnssMeasurement meas;
        meas.timestamp = msg->header.stamp.toSec();

        Eigen::Vector3d pose_ecef(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Vector3d vel_ecef(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

        if (!this->hasEnuReference()) {
            Eigen::Vector3d pose_llh = m_gnss_tools_.ecef2llh(pose_ecef);
            // setEnuReference expects (lat, lon, alt)
            this->setEnuReference(pose_llh(1), pose_llh(0), pose_llh(2));
            ROS_INFO("OdometryParser: Set GPS reference from first ECEF message.");
        }

        meas.position = m_gnss_tools_.ecef2enu(this->enu_ref_, pose_ecef);
        meas.velocity = m_gnss_tools_.ecefVelocity2enu(this->enu_ref_, vel_ecef);
        meas.position_valid = true;
        meas.velocity_valid = true;
        
        // Extract standard deviation from covariance matrix (sqrt of diagonal)
        // Pose covariance: indices 0, 7, 14 for x, y, z variance
        meas.position_std_dev << sqrt(msg->pose.covariance[0]),
                                 sqrt(msg->pose.covariance[7]),
                                 sqrt(msg->pose.covariance[14]);
        
        // Twist covariance: indices 0, 7, 14 for vx, vy, vz variance
        meas.velocity_std_dev << sqrt(msg->twist.covariance[0]),
                                 sqrt(msg->twist.covariance[7]),
                                 sqrt(msg->twist.covariance[14]);

        meas.orientation_valid = false; // Orientation in Odometry from ECEF is not typically used
        return meas;
    }
private:
    // 每个解析器实例都拥有自己的工具对象
    GNSS_Tools m_gnss_tools_; 
};


#endif // GNSS_PARSER_H