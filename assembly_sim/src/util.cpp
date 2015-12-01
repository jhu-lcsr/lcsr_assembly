#include "util.h"

namespace assembly_sim {
  /************************************************************************************/
  /*                                Helper Functions                                  */
  /************************************************************************************/

  // helper function to create names for TF
  std::string getNameTF(const std::string &ns, const std::string &joint)
  {
    std::stringstream ss;
    ss << ns << "/" << joint;
    return ss.str();
  }

  // Convert pose types
  void to_kdl(const sdf::ElementPtr &pose_elem, KDL::Frame &frame)
  {
    sdf::Pose pose;
    pose_elem->GetValue()->Get(pose);

    //std::string pose_str;
    //pose_elem->GetValue()->Get(pose_str);

    frame = KDL::Frame(
        KDL::Rotation::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w),
        KDL::Vector(pose.pos.x, pose.pos.y, pose.pos.z));

    //gzwarn<<"string of joint pose: "<<pose_str<<std::endl<<frame<<std::endl;
  }

  void to_kdl(const gazebo::math::Pose &pose, KDL::Frame &frame)
  {
    frame = KDL::Frame(
        KDL::Rotation::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w),
        KDL::Vector(pose.pos.x, pose.pos.y, pose.pos.z));
  }

  void to_kdl(const gazebo::math::Vector3 &vector3, KDL::Vector &vector)
  {
    vector.data[0] = vector3.x;
    vector.data[1] = vector3.y;
    vector.data[2] = vector3.z;
  }

  void to_tf(const gazebo::math::Pose &pose, tf::Transform &frame)
  {
    frame.setRotation( tf::Quaternion(
            pose.rot.x,
            pose.rot.y,
            pose.rot.z,
            pose.rot.w));
    frame.setOrigin( tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z) );
  }

  void to_gazebo(const KDL::Frame &frame, gazebo::math::Pose &pose)
  {
    pose = gazebo::math::Pose(
        gazebo::math::Vector3(frame.p.data[0], frame.p.data[1], frame.p.data[2]),
        gazebo::math::Quaternion());
    frame.M.GetQuaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
  }

  void to_gazebo(const KDL::Wrench &wrench, gazebo::math::Vector3 &force, gazebo::math::Vector3 &torque)
  {
    force.x = wrench.force.x();
    force.y = wrench.force.y();
    force.z = wrench.force.z();

    torque.x = wrench.torque.x();
    torque.y = wrench.torque.y();
    torque.z = wrench.torque.z();
  }

  void to_eigen(const gazebo::math::Vector3 &vector3, Eigen::Vector3d &vector3d) {
    for(int i=0; i<3; i++) {
      vector3d[i] = vector3[i];
    }
  }

  // Complete an SDF xml snippet into a model
  std::string complete_sdf(const std::string &incomplete_sdf)
  {
    return std::string("<sdf version=\"1.4\">\n<model name=\"template\">\n" + incomplete_sdf + "\n</model>\n</sdf>");
  }

}
