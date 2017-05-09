#ifndef __LCSR_ASSEMBLY_ASSEMBLY_SIM_UTIL_H__
#define __LCSR_ASSEMBLY_ASSEMBLY_SIM_UTIL_H__

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

// tf headers
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

// visualize mates in rviz gui
#include <visualization_msgs/MarkerArray.h>

// send information to ros
#include <assembly_msgs/MateList.h>

#include <kdl/frames_io.hpp>

namespace assembly_sim {
  // helper function to create names for TF
  std::string getNameTF(const std::string &ns, const std::string &joint);

  // Convert geometric types
  void to_kdl(const sdf::ElementPtr &pose_elem, KDL::Frame &frame);

  void to_kdl(const gazebo::math::Pose &pose, KDL::Frame &frame);

  void to_kdl(const gazebo::math::Vector3 &vector3, KDL::Vector &vector);

  void to_tf(const gazebo::math::Pose &pose, tf::Transform &frame);

  void to_gazebo(const KDL::Frame &frame, gazebo::math::Pose &pose);

  void to_gazebo(const KDL::Wrench &wrench, gazebo::math::Vector3 &force, gazebo::math::Vector3 &torque);

  void to_eigen(const gazebo::math::Vector3 &vector3, Eigen::Vector3d &vector3d);

  // Complete an SDF xml snippet into a model
  std::string complete_sdf(const std::string &incomplete_sdf);
}

#endif // ifndef __LCSR_ASSEMBLY_ASSEMBLY_SIM_UTIL_H__
