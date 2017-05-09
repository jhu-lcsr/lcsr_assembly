#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <boost/make_shared.hpp>
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

#include "assembly_soup_plugin.h"

/************************************************************************************/
/*                                Helper Functions                                  */
/************************************************************************************/

// helper function to create names for TF
static inline std::string getNameTF(const std::string &ns, const std::string &joint)
{
  std::stringstream ss;
  ss << ns << "/" << joint;
  return ss.str();
}

// Convert pose types
static void to_kdl(const sdf::ElementPtr &pose_elem, KDL::Frame &frame)
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

static void to_kdl(const gazebo::math::Pose &pose, KDL::Frame &frame)
{
  frame = KDL::Frame(
      KDL::Rotation::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w),
      KDL::Vector(pose.pos.x, pose.pos.y, pose.pos.z));
}

static void to_kdl(const gazebo::math::Vector3 &vector3, KDL::Vector &vector)
{
  vector.data[0] = vector3.x;
  vector.data[1] = vector3.y;
  vector.data[2] = vector3.z;
}

static void to_tf(const gazebo::math::Pose &pose, tf::Transform &frame)
{
  frame.setRotation( tf::Quaternion(
          pose.rot.x,
          pose.rot.y,
          pose.rot.z,
          pose.rot.w));
  frame.setOrigin( tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z) );
}

static void to_gazebo(const KDL::Frame &frame, gazebo::math::Pose &pose)
{
  pose = gazebo::math::Pose(
      gazebo::math::Vector3(frame.p.data[0], frame.p.data[1], frame.p.data[2]),
      gazebo::math::Quaternion());
  frame.M.GetQuaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
}

// Complete an SDF xml snippet into a model
static std::string complete_sdf(const std::string &incomplete_sdf)
{
  return std::string("<sdf version=\"1.4\">\n<model name=\"template\">\n" + incomplete_sdf + "\n</model>\n</sdf>");
}


/************************************************************************************/
/*                                Assembly Sim                                      */
/************************************************************************************/

namespace assembly_sim
{
  Mate::Mate(
      gazebo::physics::ModelPtr gazebo_model,
      MatePointPtr female_mate_point_,
      MatePointPtr male_mate_point_,
      AtomPtr female_atom,
      AtomPtr male_atom) :
    female(female_atom),
    male(male_atom),
    female_mate_point(female_mate_point_),
    male_mate_point(male_mate_point_),
    joint_sdf(),
    joint()
  {
    std::cout << male_atom->link->GetName() << std::endl;
    std::cout << female_atom->link->GetName() << std::endl;
    gzwarn<<"Creating joint for mate type: "
      <<female_mate_point->model->type<<" "
      <<female_atom->link->GetName()
      <<" -> "
      <<male_atom->link->GetName()
      <<std::endl;

    // Create a new joint
    MateModelPtr mate_model = female_mate_point->model;

    // Get the joint type
    std::string joint_type;
    mate_model->joint_template->GetAttribute("type")->Get(joint_type);

    // Customize the joint sdf template
    joint_sdf = boost::make_shared<sdf::Element>();
    joint_sdf->Copy(mate_model->joint_template);
    joint_sdf->GetAttribute("name")->Set(
        str( boost::format("%s_m%0d_to_%s_m%0d")
             % female_atom->link->GetName()
             % female_mate_point->id
             % male_atom->link->GetName()
             % male_mate_point->id));
    joint_sdf->GetElement("parent")->GetValue()->Set(female_atom->link->GetName());
    joint_sdf->GetElement("child")->GetValue()->Set(male_atom->link->GetName());

    gazebo::math::Pose pose;
    to_gazebo(male_mate_point->pose, pose);
    joint_sdf->GetElement("pose")->GetValue()->Set(pose);

    //gzwarn<<"joint sdf:\n\n"<<joint_sdf->ToString(">>")<<std::endl;

    // Construct the actual joint between these two atom links
    joint = gazebo_model->GetWorld()->GetPhysicsEngine()->CreateJoint(joint_type, gazebo_model);
    joint->SetModel(gazebo_model);

    // Load joint description from SDF
    //  - sets parend a child links
    //  - sets the anchor pose
    //  - loads sensor elements
    joint->Load(joint_sdf);

    // Initialize joint
    //  - sets axis orientation
    //  - sets axis limits
    //  - attaches parent and child via this joint
    joint->Init();

    // Joints should initially be detached
    joint->Detach();
  }

  AssemblySoup::AssemblySoup() :
    mate_id_counter_(0),
    atom_id_counter_(0),
    tf_world_frame_("world"),
    broadcast_tf_(false),
    publish_active_mates_(false),
    last_tick_(0),
    updates_per_second_(10),
    running_(false)
  {
  }

  void AssemblySoup::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model_ = _parent;
    this->sdf_ = _sdf;

    // Get TF configuration
    sdf::ElementPtr broadcast_elem = _sdf->GetElement("tf_world_frame");
    if (broadcast_elem) {
      broadcast_elem->GetValue()->Get(tf_world_frame_);
      gzwarn<<"Broadcasting TF frames for joints relative to \""<<tf_world_frame_<<"\""<<std::endl;
      broadcast_tf_ = true;

      // set up publishers for visualization
      ros::NodeHandle nh;
      male_mate_pub_ = nh.advertise<visualization_msgs::MarkerArray>("male_mate_points",1000);
      female_mate_pub_ = nh.advertise<visualization_msgs::MarkerArray>("female_mate_points",1000);
    }

    // are we going to publish ros messages describing mate status?
    if(_sdf->HasElement("publish_active_mates")) {
      sdf::ElementPtr publish_mate_elem = _sdf->GetElement("publish_active_mates");
      publish_mate_elem->GetValue()->Get(publish_active_mates_);
      if (publish_active_mates_) {
        ros::NodeHandle nh;
        active_mates_pub_ = nh.advertise<assembly_msgs::MateList>("active_mates",1000);
        gzwarn << "Publishing active mates!" << std::endl;
      } else {
        gzwarn << "Not publishing active mates!" << std::endl;
      }
    } else {
      gzwarn<<"No \"publish_active_mates\" element."<<std::endl;
    } 

    if(_sdf->HasElement("updates_per_second")) {
      sdf::ElementPtr updates_per_second_elem = _sdf->GetElement("updates_per_second");
      updates_per_second_elem->GetValue()->Get(updates_per_second_);
    }

    // Get the description of the mates in this soup
    sdf::ElementPtr mate_elem = _sdf->GetElement("mate_model");

    while(mate_elem && mate_elem->GetName() == "mate_model")
    {
      // Create a new mate model
      std::string model;
      if(mate_elem->HasAttribute("model")) {
        mate_elem->GetAttribute("model")->Get(model);
      } else {
        gzerr<<"ERROR: no mate model type for mate model"<<std::endl;
        return;
      }

      // Determine the type of mate model
      MateModelPtr mate_model;
      if(model == "proximity") {
        mate_model = boost::make_shared<ProximityMateModel>();
      } else if(model == "dipole") {
        mate_model = boost::make_shared<DipoleMateModel>();
      } else {
        gzerr<<"ERROR: \""<<model<<"\" is not a valid model type"<<std::endl;
        return;
      }

      // Get the model name
      if(mate_elem->HasAttribute("type")) {
        mate_elem->GetAttribute("type")->Get(mate_model->type);
        gzlog<<"Adding mate model for "<<mate_model->type<<std::endl;
      } else {
        gzerr<<"ERROR: no mate type for mate model"<<std::endl;
        return;
      }

      // Get the mate template joint
      mate_model->joint_template_sdf = boost::make_shared<sdf::SDF>();
      sdf::init(sdf::SDFPtr(mate_model->joint_template_sdf));
      sdf::readString(complete_sdf(mate_elem->GetElement("joint")->ToString("")), mate_model->joint_template_sdf);
      mate_model->joint_template = mate_model->joint_template_sdf->root->GetElement("model")->GetElement("joint");

      // Get the mate symmetries
      sdf::ElementPtr symmetry_elem = mate_elem->GetElement("symmetry");
      if(symmetry_elem)
      {
        sdf::ElementPtr rot_elem = symmetry_elem->GetElement("rot");

        if(rot_elem)
        {
          sdf::Vector3 rot_symmetry;
          rot_elem->GetValue()->Get(rot_symmetry);

          // compute symmetries
          const double x_step = M_PI*2.0/rot_symmetry.x;
          const double y_step = M_PI*2.0/rot_symmetry.y;
          const double z_step = M_PI*2.0/rot_symmetry.z;

          for(double ix=0; ix < rot_symmetry.x; ix++)
          {
            KDL::Rotation Rx = KDL::Rotation::RotX(ix * x_step);
            for(double iy=0; iy < rot_symmetry.y; iy++)
            {
              KDL::Rotation Ry = KDL::Rotation::RotY(iy * y_step);
              for(double iz=0; iz < rot_symmetry.z; iz++)
              {
                KDL::Rotation Rz = KDL::Rotation::RotZ(iz * z_step);
                mate_model->symmetries.push_back(KDL::Frame(Rx*Ry*Rz, KDL::Vector(0,0,0)));
              }
            }
          }
        }
      }

      // Load the parameters from the mate model
      mate_model->load(mate_elem);

      // Add the identity if no symmetries were added
      if(mate_model->symmetries.size() == 0) {
        mate_model->symmetries.push_back(KDL::Frame());
      }

      // Store this mate model
      mate_models_[mate_model->type] = mate_model;

      // Get the next atom element
      mate_elem = mate_elem->GetNextElement(mate_elem->GetName());
    }

    // Get the description of the atoms in this soup
    sdf::ElementPtr atom_elem = _sdf->GetElement("atom_model");

    while(atom_elem && atom_elem->GetName() == "atom_model")
    {
      // Create a new atom
      AtomModelPtr atom_model = boost::make_shared<AtomModel>();
      atom_elem->GetAttribute("type")->Get(atom_model->type);

      // Get the atom mate points
      sdf::ElementPtr mate_elem = atom_elem->GetElement("mate_point");
      while(mate_elem)
      {
        std::string type;
        std::string gender;
        KDL::Frame base_pose;

        mate_elem->GetAttribute("type")->Get(type);
        mate_elem->GetAttribute("gender")->Get(gender);
        to_kdl(mate_elem->GetElement("pose"), base_pose);

        gzwarn<<"Adding mate point type: "<<type<<" gender: "<<gender<<" at: "<<base_pose<<std::endl;

        MateModelPtr mate_model = mate_models_[type];
        MatePointPtr mate_point;

        if(boost::iequals(gender, "female")) {
          for(std::vector<KDL::Frame>::iterator pose_it = mate_model->symmetries.begin();
              pose_it != mate_model->symmetries.end();
              ++pose_it)
          {
            mate_point = boost::make_shared<MatePoint>();
            mate_point->model = mate_model;
            mate_point->pose = base_pose * (*pose_it);
            mate_point->id =
              atom_model->female_mate_points.size()
              + atom_model->male_mate_points.size();

            gzwarn<<"Adding female mate point "<<atom_model->type<<"#"<<mate_point->id<<" pose: "<<std::endl<<mate_point->pose<<std::endl;

            atom_model->female_mate_points.push_back(mate_point);
          }
        } else if(boost::iequals(gender, "male")) {
          mate_point = boost::make_shared<MatePoint>();
          mate_point->model = mate_model;
          mate_point->pose = base_pose;
            mate_point->id =
              atom_model->female_mate_points.size()
              + atom_model->male_mate_points.size();

          gzwarn<<"Adding male mate point "<<atom_model->type<<"#"<<mate_point->id<<" pose: "<<std::endl<<mate_point->pose<<std::endl;

          atom_model->male_mate_points.push_back(mate_point);
        } else {
          gzerr<<"Unknown gender: "<<gender<<std::endl;
        }

        // Get the next mate point element
        mate_elem = mate_elem->GetNextElement(mate_elem->GetName());
      }

      // Store this atom
      atom_models_[atom_model->type] = atom_model;

      // Get the next atom element
      atom_elem = atom_elem->GetNextElement(atom_elem->GetName());
    }

    // Extract the links from the model
    gazebo::physics::Link_V assembly_links = this->model_->GetLinks();
    for(gazebo::physics::Link_V::iterator it=assembly_links.begin();
        it != assembly_links.end();
        ++it)
    {
      // Create new atom
      AtomPtr atom = boost::make_shared<Atom>();
      atom->link = *it;

      // Determine the atom type from the link name
      for(std::map<std::string, AtomModelPtr>::iterator model_it=atom_models_.begin();
          model_it != atom_models_.end();
          ++model_it)
      {
        if(atom->link->GetName().find(model_it->second->type) == 0) {
          atom->model = model_it->second;
          break;
        }
      }

      atoms_.push_back(atom);
    }

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&AssemblySoup::OnUpdate, this, _1));
  }

  void AssemblySoup::DoProximityCheck() {

    static tf::TransformBroadcaster br;

    assembly_msgs::MateList mates_msg;

    unsigned int iter = 0;
    // Iterate over all atoms
    for(std::vector<AtomPtr>::iterator it_fa = atoms_.begin();
        it_fa != atoms_.end();
        ++it_fa,++iter)
    {
      AtomPtr female_atom = *it_fa;

      // Get the female atom frame
      KDL::Frame female_atom_frame;
      to_kdl(female_atom->link->GetWorldPose(), female_atom_frame);

      // Construct some names for use with TF
      const std::string atom_name = str(
          boost::format("%s")
          % female_atom->link->GetName());
      const std::string link_name = str(
          boost::format("%s/%s")
          % atom_name
          % female_atom->model->type);

      // Broadcast TF frames for this link
      if(broadcast_tf_)
      {

        //gzwarn<<"broadcasting tf/marker info"<<std::endl;

        tf::Transform tf_frame;
        visualization_msgs::MarkerArray male_mate_markers;
        visualization_msgs::MarkerArray female_mate_markers;

        // Broadcast a tf frame for this link
        to_tf(female_atom->link->GetWorldPose(), tf_frame);
        br.sendTransform(
            tf::StampedTransform(
                tf_frame,
                ros::Time::now(),
                tf_world_frame_,
                link_name));

        // Broadcast all male mate points for this atom
        for(std::vector<MatePointPtr>::iterator it_mmp = female_atom->model->male_mate_points.begin();
            it_mmp != female_atom->model->male_mate_points.end();
            ++it_mmp)
        {
          MatePointPtr male_mate_point = *it_mmp;

          const std::string male_mate_point_name = str(
              boost::format("%s/male_%d")
              % atom_name
              % male_mate_point->id);

          tf::poseKDLToTF(male_mate_point->pose,tf_frame);
          br.sendTransform(
              tf::StampedTransform(
                  tf_frame,
                  ros::Time::now(),
                  link_name,
                  male_mate_point_name));

          visualization_msgs::Marker mate_marker;
          mate_marker.header.frame_id = male_mate_point_name;
          mate_marker.header.stamp = ros::Time(0);
          mate_marker.type = mate_marker.CUBE;
          mate_marker.action = mate_marker.ADD;
          mate_marker.id = (iter * 100) + male_mate_point->id;
          mate_marker.scale.x = 0.02;
          mate_marker.scale.y = 0.02;
          mate_marker.scale.z = 0.01;
          mate_marker.color.r = 1.0;
          mate_marker.color.g = 0.0;
          mate_marker.color.b = 0.0;
          mate_marker.color.a = 0.25;
          male_mate_markers.markers.push_back(mate_marker);
        }

        // Broadcast all female mate points for this atom
        for(std::vector<MatePointPtr>::iterator it_fmp = female_atom->model->female_mate_points.begin();
            it_fmp != female_atom->model->female_mate_points.end();
            ++it_fmp)
        {
          MatePointPtr female_mate_point = *it_fmp;

          const std::string female_mate_point_name = str(
              boost::format("%s/female_%d")
              % atom_name
              % female_mate_point->id);

          tf::poseKDLToTF(female_mate_point->pose, tf_frame);
          br.sendTransform(
              tf::StampedTransform(
                  tf_frame,
                  ros::Time::now(),
                  link_name,
                  female_mate_point_name));

          visualization_msgs::Marker mate_marker;
          mate_marker.header.frame_id = female_mate_point_name;
          mate_marker.header.stamp = ros::Time(0);
          mate_marker.type = mate_marker.CUBE;
          mate_marker.action = mate_marker.ADD;
          mate_marker.id = (iter * 100) + female_mate_point->id;
          mate_marker.scale.x = 0.02;
          mate_marker.scale.y = 0.02;
          mate_marker.scale.z = 0.01;
          mate_marker.color.r = 0.0;
          mate_marker.color.g = 0.0;
          mate_marker.color.b = 1.0;
          mate_marker.color.a = 0.25;
          female_mate_markers.markers.push_back(mate_marker);
        }

        male_mate_pub_.publish(male_mate_markers);
        female_mate_pub_.publish(female_mate_markers);
      }

      // Iterate over all female mate points of female link
      for(std::vector<MatePointPtr>::iterator it_fmp = female_atom->model->female_mate_points.begin();
          it_fmp != female_atom->model->female_mate_points.end();
          ++it_fmp)
      {
        MatePointPtr female_mate_point = *it_fmp;

        // Compute the world pose of the female mate frame
        KDL::Frame female_mate_frame = female_atom_frame * female_mate_point->pose;

        // Iterate over all other atoms
        for(std::vector<AtomPtr>::iterator it_ma = atoms_.begin();
            it_ma != atoms_.end();
            ++it_ma)
        {
          AtomPtr male_atom = *it_ma;

          // You can't mate with yourself
          if(male_atom == female_atom) { continue; }

          KDL::Frame male_atom_frame;
          to_kdl(male_atom->link->GetWorldPose(), male_atom_frame);

          // Iterate over all male mate points of male link
          for(std::vector<MatePointPtr>::iterator it_mmp = male_atom->model->male_mate_points.begin();
              it_mmp != male_atom->model->male_mate_points.end();
              ++it_mmp)
          {
            MatePointPtr male_mate_point = *it_mmp;

            // Skip if the mates are incompatible
            if(female_mate_point->model != male_mate_point->model) { continue; }

            // Get the mate between these two mate points
            MatePtr mate;
            MateModelPtr mate_model = female_mate_point->model;
            mate_table_t::iterator mtf = mate_table_.find(female_mate_point);

            if(mtf == mate_table_.end())
            {
              // This female mate point needs to be added
              std::cout<<"adding female"<<std::endl;
              mate_point_map_t mate_point_map;
              mate = boost::make_shared<Mate>(
                  model_,
                  female_mate_point,
                  male_mate_point,
                  female_atom,
                  male_atom);
              mate_point_map[male_mate_point] = mate;
              mate_table_[female_mate_point] = mate_point_map;
              std::cout<<"done adding female"<<std::endl;
            }
            else if(mtf->second.find(male_mate_point) == mtf->second.end())
            {
              // This male mate point needs to be added
              std::cout<<"adding male"<<std::endl;
              mate = boost::make_shared<Mate>(
                  model_,
                  female_mate_point,
                  male_mate_point,
                  female_atom,
                  male_atom);

              mtf->second[male_mate_point] = mate;
              std::cout<<"done  adding male"<<std::endl;
            }
            else
            {
              // This mate pair is already in the table
              mate = mtf->second.at(male_mate_point);
            }

            // Schedule mates to detach and attach
            mate->model->update(mate, mates_to_detach, mates_to_affect, mates_to_attach);

            // Broadcast the TF frame for this joint
            if (broadcast_tf_ and mate->joint->GetParent() and mate->joint->GetChild())
            {
              tf::Transform tf_joint_frame;
              //to_kdl(male_atom->link->GetWorldPose() * mate->joint->GetInitialAnchorPose(), tf_frame);
              //to_tf(mate->joint->GetWorldPose(), tf_frame);

              gazebo::math::Vector3 anchor = mate->joint->GetAnchor(0);

              KDL::Frame joint_frame = KDL::Frame(
                  male_mate_frame.M,
                  KDL::Vector(anchor.x, anchor.y, anchor.z));
              tf::poseKDLToTF(joint_frame, tf_joint_frame);

              br.sendTransform(
                  tf::StampedTransform(
                      tf_joint_frame,
                      ros::Time::now(),
                      tf_world_frame_,
                      mate->joint->GetName()));
            }
          }
        }
      }
    }

    if (publish_active_mates_) {
      active_mates_pub_.publish(mates_msg);
    }

  }

  AssemblySoup::~AssemblySoup() {
    running_ = false;
    check_thread_.join();
  }

  void AssemblySoup::CheckProximityLoop() {

    std::cout << "Collision thread running!" << std::endl;

    while (running_) {

      clock_t current_tick = clock();

      if ((float)(current_tick - last_tick_) / CLOCKS_PER_SEC > 1.0 / updates_per_second_) {

        update_mutex_.lock();

        // clear mates
        mates_to_attach.resize(0);
        mates_to_detach.resize(0);

        DoProximityCheck();
        update_mutex_.unlock();

        last_tick_ = clock();
      }
    }
  }

  // Called by the world update start event
  // This is where the logic that connects and updates joints needs to happen
  void AssemblySoup::OnUpdate(const gazebo::common::UpdateInfo & /*_info*/)
  {

    if (!running_) {

      DoProximityCheck();

      std::cout << "Starting thread..." << std::endl;
      check_thread_ = boost::thread(boost::bind(&AssemblySoup::CheckProximityLoop, this));
      running_ = true;
      std::cout << "Started." <<std::endl;
    }

    if (update_mutex_.try_lock()) {

      for (std::vector<MatePtr>::iterator it = mates_to_detach.begin();
           it != mates_to_detach.end();
           ++it)
      {
        MatePtr mate = *it;
        gzwarn<<" --- demating "<<mate->joint->GetName()<<std::endl;
        mate->model->detach(mate);
      }
      for (std::vector<MatePtr>::iterator it = mates_to_attach.begin();
           it != mates_to_attach.end();
           ++it)
      {
        MatePtr mate = *it;
        gzwarn<<" --- mating "<<mate->joint->GetName()<<std::endl;
        mate->model->attach(mate);
      }

      update_mutex_.unlock();
    }
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AssemblySoup);
}

