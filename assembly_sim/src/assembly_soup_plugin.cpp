#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

#include <kdl/frames_io.hpp>

#include "assembly_soup_plugin.h"

/************************************************************************************/
/*                                Helper Functions                                  */
/************************************************************************************/

// helper function to create names for TF
static inline std::string getNameTF(const std::string &ns, const std::string &joint) {
  std::stringstream ss;
  ss << ns << "/" << joint;
  return ss.str();
}

static void to_kdl(const sdf::ElementPtr pose_elem, KDL::Frame &frame)
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

static void to_tf(const gazebo::math::Pose &pose, tf::Transform &frame) {
  frame.setRotation( tf::Quaternion(pose.rot.x,
                                        pose.rot.y,
                                        pose.rot.z,
                                        pose.rot.w) );
  frame.setOrigin( tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z) );
}

static void to_kdl(const gazebo::math::Pose &pose, KDL::Frame &frame)
{
  frame = KDL::Frame(
      KDL::Rotation::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w),
      KDL::Vector(pose.pos.x, pose.pos.y, pose.pos.z));
}

static void to_gazebo(const KDL::Frame &frame, gazebo::math::Pose &pose)
{
  pose = gazebo::math::Pose(
      gazebo::math::Vector3(frame.p.data[0], frame.p.data[1], frame.p.data[2]),
      gazebo::math::Quaternion());
  frame.M.GetQuaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
}

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
      MatePointModelPtr female_mate_point_model,
      MatePointModelPtr male_mate_point_model,
      AtomPtr female_atom,
      AtomPtr male_atom) :
    joint_sdf(),
    joint()
  {
    gzwarn<<"Creating joint for mate type: "
      <<female_mate_point_model->model->type<<" "
      <<female_atom->link->GetName()
      <<" -> "
      <<male_atom->link->GetName()
      <<std::endl;

    // Create a new joint
    // TODO: make sure the joint is initialized with the joint template
    // TODO: make sure it is created detached
    // TODO: make sure it is created in the right location
    MateModelPtr mate_model = female_mate_point_model->model;

    // Get the joint type
    std::string joint_type;
    mate_model->joint_template->GetAttribute("type")->Get(joint_type);

    // Customize the joint sdf template
    joint_sdf = boost::make_shared<sdf::Element>();
    joint_sdf->Copy(mate_model->joint_template);
    joint_sdf->GetAttribute("name")->Set(
        str( boost::format("%s_m%0d_to_%s_m%0d") 
             % female_atom->link->GetName()
             % female_mate_point_model->id 
             % male_atom->link->GetName()
             % male_mate_point_model->id));
    joint_sdf->GetElement("parent")->GetValue()->Set(female_atom->link->GetName());
    joint_sdf->GetElement("child")->GetValue()->Set(male_atom->link->GetName());

    gazebo::math::Pose pose;
    to_gazebo(male_mate_point_model->pose, pose);
    joint_sdf->GetElement("pose")->GetValue()->Set(pose);

    gzwarn<<"joint sdf:\n\n"<<joint_sdf->ToString(">>")<<std::endl;

    // Construct the actual link between these two atoms
    joint = gazebo_model->GetWorld()->GetPhysicsEngine()->CreateJoint(joint_type, gazebo_model);
    joint->SetModel(gazebo_model);
    joint->Load(joint_sdf);
    joint->Detach();
  }

  AssemblySoup::AssemblySoup() :
    mate_id_counter_(0),
    atom_id_counter_(0),
    max_trans_err_(0.01),
    max_rot_err_(0.1),
    tf_world_frame_("world"),
    broadcast_tf_(false)
  {

  }

  void AssemblySoup::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model_ = _parent;
    this->sdf_ = _sdf;


    // store mate point error
    // from configuration in files
    sdf::ElementPtr trans_err_elem = _sdf->GetElement("trans_mate_err");
    if (trans_err_elem) {
      double trans_mate_err;
      bool _err_res = trans_err_elem->GetValue()->Get(trans_mate_err);
      gzwarn<<"Setting mate translational error="<<trans_mate_err<<std::endl;
      this->max_trans_err_ = trans_mate_err;
    }
    sdf::ElementPtr rot_err_elem = _sdf->GetElement("rot_mate_err");
    if (rot_err_elem) {
      double rot_mate_err;
      bool _err_res = rot_err_elem->GetValue()->Get(rot_mate_err);
      gzwarn<<"Setting mate rotational error="<<rot_mate_err<<std::endl;
      this->max_rot_err_ = rot_mate_err;
    }
    sdf::ElementPtr broadcast_elem = _sdf->GetElement("tf_world_frame");
    if (broadcast_elem) {
      broadcast_elem->GetValue()->Get(tf_world_frame_);
      gzwarn<<"Broadcasting TF frames for joints relative to \""<<tf_world_frame_<<"\""<<std::endl;
      broadcast_tf_ = true;
    }

    // Get the description of the mates in this soup
    sdf::ElementPtr mate_elem = _sdf->GetElement("mate_model");

    while(mate_elem && mate_elem->GetName() == "mate_model")
    {
      // Create a new mate
      MateModelPtr mate_model = boost::make_shared<MateModel>();
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

#if 0
      // Get the atom link
      atom_model->link_template = boost::make_shared<sdf::Element>();
      sdf::readString(atom_elem->GetElement("link")->ToString(""), atom_model->link_template);

      // Get the atom template link
      atom_model->link_template_sdf = boost::make_shared<sdf::SDF>();
      sdf::init(sdf::SDFPtr(atom_model->link_template_sdf));
      sdf::readString(complete_sdf(atom_elem->GetElement("link")->ToString("")), atom_model->link_template_sdf);
      atom_model->link_template = atom_model->link_template_sdf->root->GetElement("model")->GetElement("link");
#endif

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
        MatePointModelPtr mate_point_model;

        if(boost::iequals(gender, "female")) {
          for(std::vector<KDL::Frame>::iterator pose_it = mate_model->symmetries.begin();
              pose_it != mate_model->symmetries.end();
              ++pose_it)
          {
            mate_point_model = boost::make_shared<MatePointModel>();
            mate_point_model->model = mate_model;
            mate_point_model->pose = (*pose_it) * base_pose;
            mate_point_model->id =
              atom_model->female_mate_points.size()
              + atom_model->male_mate_points.size();

            gzwarn<<"Adding female mate point "<<atom_model->type<<"#"<<mate_point_model->id<<" pose: "<<std::endl<<mate_point_model->pose<<std::endl;

            atom_model->female_mate_points.push_back(mate_point_model);
          }
        } else if(boost::iequals(gender, "male")) {
          mate_point_model = boost::make_shared<MatePointModel>();
          mate_point_model->model = mate_model;
          mate_point_model->pose = base_pose;
            mate_point_model->id =
              atom_model->female_mate_points.size()
              + atom_model->male_mate_points.size();

          gzwarn<<"Adding male mate point "<<atom_model->type<<"#"<<mate_point_model->id<<" pose: "<<std::endl<<mate_point_model->pose<<std::endl;

          atom_model->male_mate_points.push_back(mate_point_model);
        } else {
          gzerr<<"Unknown gender: "<<gender<<std::endl;
        }

        // Get the next mate point element
        mate_elem = atom_elem->GetNextElement(mate_elem->GetName());
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

      // Add the mate points
      for(std::vector<MatePointModelPtr>::iterator mpm_it=atom->model->female_mate_points.begin();
          mpm_it != atom->model->female_mate_points.end();
          ++mpm_it)
      {
        MatePointPtr mate_point = boost::make_shared<MatePoint>();
        mate_point->model = *mpm_it;
        atom->female_mate_points.push_back(mate_point);
      }

      for(std::vector<MatePointModelPtr>::iterator mpm_it=atom->model->male_mate_points.begin();
          mpm_it != atom->model->male_mate_points.end();
          ++mpm_it)
      {
        MatePointPtr mate_point = boost::make_shared<MatePoint>();
        mate_point->model = *mpm_it;
        atom->male_mate_points.push_back(mate_point);
      }

      atoms_.push_back(atom);
    }

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&AssemblySoup::OnUpdate, this, _1));
  }

  // Called by the world update start event
  // This is where the logic that connects and updates joints needs to happen
  void AssemblySoup::OnUpdate(const gazebo::common::UpdateInfo & /*_info*/)
  {

    static tf::TransformBroadcaster br;

    unsigned int link_idx = 0;
    // Iterate over all atoms
    for(std::vector<AtomPtr>::iterator it_fa = atoms_.begin();
        it_fa != atoms_.end();
        ++it_fa,++link_idx)
    {
      AtomPtr female_atom = *it_fa;
      //gzwarn<<" - female: "<<female_atom->link->GetName()<<std::endl;

      KDL::Frame female_atom_frame;
      to_kdl(female_atom->link->GetWorldPose(), female_atom_frame);

      std::stringstream ss;
      ss << "link" << link_idx;
      std::string link_name = ss.str();
      ss << "/" << female_atom->model->type;
      std::string body_name = ss.str();

      if(broadcast_tf_) {
        std::stringstream ss;
        tf::Transform tf_frame;
        to_tf(female_atom->link->GetWorldPose(), tf_frame);
        br.sendTransform(tf::StampedTransform(tf_frame,
                                              ros::Time::now(),
                                              tf_world_frame_,
                                              body_name));
      }


      // print out all mate points
      // done in a separate loop so we don't publish too many things...
      // this would be very wasteful if we buried it in the innermost for-loop
      if(broadcast_tf_) {
        unsigned int male_idx = 0;
        for(std::vector<MatePointPtr>::iterator it_mmp = female_atom->male_mate_points.begin();
            it_mmp != female_atom->male_mate_points.end();
            ++it_mmp,++male_idx)
        {
          MatePointPtr male_mate_point = *it_mmp;

          std::stringstream ss;
          ss << link_name << "/male" << male_idx;

          tf::Transform tf_frame;
          tf::poseKDLToTF(male_mate_point->model->pose,tf_frame);
          br.sendTransform(tf::StampedTransform(tf_frame,
                                                ros::Time::now(),
                                                body_name,
                                                ss.str()));
        }
      }

      unsigned int female_idx = 0;
      // Iterate over all female mate points of female link
      for(std::vector<MatePointPtr>::iterator it_fmp = female_atom->female_mate_points.begin();
          it_fmp != female_atom->female_mate_points.end();
          ++it_fmp,++female_idx)
      {
        MatePointPtr female_mate_point = *it_fmp;
        if(!female_mate_point->model->model) {
          gzerr<<"MATE POINT UNDEFINED"<<std::endl;
          return;
        }
        //gzwarn<<" -- female mate: "<<female_mate_point->model->model->type<<std::endl;

        // Compute the pose of the female mate frame
        KDL::Frame female_mate_frame = female_atom_frame * female_mate_point->model->pose;

        if(broadcast_tf_) {
          std::stringstream ss;
          ss << link_name << "/female" << female_idx;
          tf::Transform tf_frame;
          tf::poseKDLToTF(female_mate_point->model->pose,tf_frame);
          br.sendTransform(tf::StampedTransform(tf_frame,
                                                ros::Time::now(),
                                                body_name,
                                                ss.str()));
        }

        // Iterate over all other atoms
        for(std::vector<AtomPtr>::iterator it_ma = atoms_.begin();
            it_ma != atoms_.end();
            ++it_ma)
        {
          AtomPtr male_atom = *it_ma;
          KDL::Frame male_atom_frame;
          to_kdl(male_atom->link->GetWorldPose(), male_atom_frame);

          // You can't mate with yourself
          if(male_atom == female_atom) { continue; }

          // Iterate over all male mate points of male link
          for(std::vector<MatePointPtr>::iterator it_mmp = male_atom->male_mate_points.begin();
              it_mmp != male_atom->male_mate_points.end();
              ++it_mmp)
          {
            MatePointPtr male_mate_point = *it_mmp;

            // Skip if the mates are incompatible
            if(female_mate_point->model->model->type != male_mate_point->model->model->type) { continue; }

            // Compute the pose of the male mate frame
            KDL::Frame male_mate_frame = male_atom_frame * male_mate_point->model->pose;

            // Get the mate between these two mate points
            MatePtr mate;
            mate_table_t::iterator mtf = mate_table_.find(female_mate_point);

            if(mtf == mate_table_.end())
            {
              // This female mate point needs to be added
              mate_point_map_t mate_point_map;
              mate = boost::make_shared<Mate>(
                  model_,
                  female_mate_point->model,
                  male_mate_point->model,
                  female_atom,
                  male_atom);
              mate_point_map[male_mate_point] = mate;
              mate_table_[female_mate_point] = mate_point_map;
            }
            else if(mtf->second.find(male_mate_point) == mtf->second.end())
            {
              // This male mate point needs to be added
              mate = boost::make_shared<Mate>(
                  model_,
                  female_mate_point->model,
                  male_mate_point->model,
                  female_atom,
                  male_atom);

              mtf->second[male_mate_point] = mate;
            }
            else
            {
              // This mate pair is already in the table
              mate = mtf->second.at(male_mate_point);
            }

            // compute twist between the two mate points
            KDL::Twist twist_err = diff(female_mate_frame, male_mate_frame);

            if(mate->joint) {
              if(mate->joint->GetParent() and mate->joint->GetChild()) {
                //gzwarn<<"Parts are mated!"<<std::endl;
                if(false) {
                  // detach joint
                  mate->joint->Detach();
                  // disable the mate
                }
              } else {
                //gzwarn<<"Parts are not mated!"<<std::endl;
                if(twist_err.vel.Norm() < max_trans_err_ and twist_err.rot.Norm() < max_rot_err_) {
                  gzwarn<<" --- from "<<female_atom->link->GetName()<<" -> "<<male_atom->link->GetName()<<std::endl;
                  gzwarn<<" --- trans twist err: "<<twist_err.vel<<std::endl;
                  gzwarn<<" --- rot twist err: "<<twist_err.rot<<std::endl;
                  gzwarn<<" --- mating."<<std::endl;

                  // attach joint
                  mate->joint->Attach(female_atom->link, male_atom->link);
                  KDL::Frame tmp;
                  to_kdl(mate->joint->GetInitialAnchorPose(),tmp);
                  gzwarn<<" --- joint pose: "<<std::endl<<tmp<<std::endl;
                  //mate->joint->Init();
                }
              }
            } else {
              //gzwarn<<"No joint for mate from "<<female_atom->link->GetName()<<" -> "<<male_atom->link->GetName()<<std::endl;
            }
          }
        }
      }
    }
  }

#if 0
  void AssemblySoup::instantiate_atom(const AtomModelPtr &atom_model, const sdf::Pose &pose)
  {
    AtomPtr atom = boost::make_shared<Atom>();

    // Fill out the link template with the instantiated information
    sdf::ElementPtr link_template(new sdf::Element());
    link_template->Copy(atom_model->link_template);

    link_template->GetAttribute("name")->Set(str(boost::format("%s_%0d") % atom_model->type % atom_id_counter_++));
    link_template->GetElement("pose")->GetValue()->Set(pose);

    gzwarn<<"instantiate: "<<link_template->ToString(">> ")<<std::endl;


    // Create a new link owned by this model
    sdf_->InsertElement(link_template);
    atom->link =
      model_->GetWorld()->GetPhysicsEngine()->CreateLink(
          boost::static_pointer_cast<gazebo::physics::Model>(model_));

    // Load the link information
    atom->link->Load(link_template);
    atom->link->Init();
    atom->link->SetEnabled(true);
    model_->GetLinks().push_back(atom->link);

    atoms_.push_back(atom);
  }
#endif

#if 0
  void mate_atoms(gazebo::physics::LinkPtr female, gazebo::physics::LinkPtr male) {

    // Instantiate joints for each female mate
    for(std::vector<Mate>::const_iterator mate=atom.female_mates.begin();
        mate != atom.female_mates.end();
        ++mate)
    {
      gazebo::physics::JointPtr joint(new gazebo::physics::Joint());

      joint->Load(mate->joint_template);
      joint->SetName(str(boost::format("%s_%0d_%s_%0d") % atom.type % n_atoms % mate->type % n_mates++));

      link->AddChildJoint(joint);
    }

  }
#endif

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AssemblySoup)
}

