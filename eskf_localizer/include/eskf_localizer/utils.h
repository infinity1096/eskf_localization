#pragma once

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <LocalCartesian.hpp>

namespace ESKF_Localization {

constexpr double kDegreeToRadian = M_PI / 180.;
constexpr double kRadianToDegree = 180. / M_PI;

inline void ConvertLLAToENU(const Eigen::Vector3d& init_lla, 
                            const Eigen::Vector3d& point_lla, 
                            Eigen::Vector3d* point_enu) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2), 
                            point_enu->data()[0], point_enu->data()[1], point_enu->data()[2]);
}

inline void ConvertENUToLLA(const Eigen::Vector3d& init_lla, 
                            const Eigen::Vector3d& point_enu,
                            Eigen::Vector3d* point_lla) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2), 
                            point_lla->data()[0], point_lla->data()[1], point_lla->data()[2]);                            
}

inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w <<  0.,   -v(2),  v(1),
          v(2),  0.,   -v(0),
         -v(1),  v(0),  0.;

    return w;
}

inline Eigen::Matrix<double,3,4> diff_q_a_qT_diff_q(const Eigen::Quaterniond& q, const Eigen::Vector3d& a){
	Eigen::Matrix<double,3,4> jacob = Eigen::Matrix<double,3,4>::Zero();
	Eigen::Vector3d v(q.x(),q.y(),q.z());

	jacob.block<3,1>(0,0) = q.w() * a + v.cross(a);
	jacob.block<3,3>(0,1) = v.transpose()*a*Eigen::Matrix3d::Identity() + v*a.transpose() - a *v.transpose() - q.w()*hat(a);
	return jacob;
}

inline Eigen::Matrix<double,3,4> diff_qT_a_q_diff_q(const Eigen::Quaterniond& q, const Eigen::Vector3d& a){
	Eigen::Matrix<double,3,4> jacob = diff_q_a_qT_diff_q(q,a);
	Eigen::Matrix4d diff_qT_diff_q = Eigen::Matrix4d::Identity();
	diff_qT_diff_q.block<3,3>(1,1) = -Eigen::Matrix3d::Identity();
	return jacob * diff_qT_diff_q;
}

inline Eigen::Matrix4d quat_l(const Eigen::Quaterniond& q){
	Eigen::Matrix4d mat;
	Eigen::Vector3d qv(q.x(),q.y(),q.z());
	mat << 0, -qv.transpose(), qv, hat(qv);
	mat += q.w() * Eigen::Matrix4d::Identity();
	return mat;
}

}  // namespace ImuGpsLocalization
