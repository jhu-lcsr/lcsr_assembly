#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "assembly_soup_plugin.h"

static void to_kdl(const sdf::ElementPtr pose_elem, KDL::Frame &frame)
{
  sdf::Pose pose;
  pose_elem->GetValue()->Get(pose);

  frame = KDL::Frame(
      KDL::Rotation::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w),
      KDL::Vector(pose.pos.x, pose.pos.y, pose.pos.z));
}

static void to_kdl(const gazebo::math::Pose pose, KDL::Frame &frame)
{
  frame = KDL::Frame(
      KDL::Rotation::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w),
      KDL::Vector(pose.pos.x, pose.pos.y, pose.pos.z));
}

namespace assembly_sim
{

  AssemblySoup::AssemblySoup() :
    mate_id_counter_(0),
    atom_id_counter_(0)
  {

  }

  void AssemblySoup::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model_ = _parent;

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

      // Get the mate joint
      mate_model->joint_template = boost::make_shared<sdf::Element>();
      sdf::readString(mate_elem->GetElement("joint")->ToString(), mate_model->joint_template);

      // Get the mate symmetries
      sdf::ElementPtr symmetry_elem = _sdf->GetElement("symmetry");
      if(symmetry_elem) {
        sdf::ElementPtr rot_elem = _sdf->GetElement("rot");

        if(rot_elem) {
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

      // Get the atom link
      atom_model->link_template = boost::make_shared<sdf::Element>();
      sdf::readString(atom_elem->GetElement("link")->ToString(), atom_model->link_template);

      // Get the atom mate points
      sdf::ElementPtr mate_elem = _sdf->GetElement("mate_point");
      while(mate_elem)
      {
        std::string type;
        std::string gender;
        KDL::Frame base_pose;

        mate_elem->GetAttribute("type")->Get(type);
        mate_elem->GetAttribute("gender")->Get(gender);
        to_kdl(mate_elem->GetElement("pose"), base_pose);

        MateModelPtr mate_model = mate_models_[type];

        for(std::vector<KDL::Frame>::iterator pose_it = mate_model->symmetries.begin();
            pose_it != mate_model->symmetries.end();
            ++pose_it)
        {
          MatePointModelPtr mate_point_model = boost::make_shared<MatePointModel>();
          mate_point_model->pose = (*pose_it) * base_pose;

          if(boost::iequals(gender, "female")) {
            atom_model->female_mate_points.push_back(mate_point_model);
          } else if(boost::iequals(gender, "male")) {
            atom_model->male_mate_points.push_back(mate_point_model);
          }
        }

        // Get the next mate point element
        mate_elem = atom_elem->GetNextElement(mate_elem->GetName());
      }

      // Store this atom
      atom_models_[atom_model->type] = atom_model;

      // Get the next atom element
      atom_elem = atom_elem->GetNextElement(atom_elem->GetName());
    }

    // Construct the initial assembly soup
    sdf::ElementPtr assembly_elem = _sdf->GetElement("assembly");
    sdf::ElementPtr assem_atom_elem = assembly_elem->GetElement("atom");

    while(assem_atom_elem && assem_atom_elem->GetName() == "atom")
    {
      // Get the instance details
      std::string type;
      sdf::Pose pose;

      assem_atom_elem->GetAttribute("type")->Get(type);
      assem_atom_elem->GetElement("pose")->GetValue()->Get(pose);

      // Instantiate the atom
      instantiate_atom(atom_models_[type], pose);

      // Get the next atom element
      assem_atom_elem = assem_atom_elem->GetNextElement(assem_atom_elem->GetName());
    }

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&AssemblySoup::OnUpdate, this, _1));
  }

  // Called by the world update start event
  void AssemblySoup::OnUpdate(const gazebo::common::UpdateInfo & /*_info*/)
  {
    // Iterate over all atoms
    for(std::vector<AtomPtr>::iterator it_fa = atoms_.begin();
        it_fa != atoms_.end();
        ++it_fa)
    {
      AtomPtr female_atom = *it_fa;

      // Iterate over all female mate points of female link
      for(std::vector<MatePointPtr>::iterator it_fmp = female_atom->female_mate_points.begin();
          it_fmp != female_atom->female_mate_points.end();
          ++it_fmp)
      {
        MatePointPtr female_mate_point = *it_fmp;

        KDL::Frame female_mate_pose;

        // Iterate over all other atoms
        for(std::vector<AtomPtr>::iterator it_ma = atoms_.begin();
            it_ma != atoms_.end();
            ++it_ma)
        {
          AtomPtr male_atom = *it_ma;

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

            KDL::Frame male_mate_pose;

            // Get the mate between these two mate points
            MatePtr mate;
            mate_table_t::iterator mtf = mate_table_.find(female_mate_point);

            if(mtf == mate_table_.end())
            {
              // This female mate point needs to be added
              mate = boost::make_shared<Mate>();
              mate_point_map_t mate_point_map;
              mate_point_map[male_mate_point] = mate;
              mate_table_[female_mate_point] = mate_point_map;
            }
            else if(mtf->second.find(male_mate_point) == mtf->second.end())
            {
              // This male mate point needs to be added
              mate = boost::make_shared<Mate>();
              mtf->second[male_mate_point] = mate;
            }
            else
            {
              // This mate pair is already in the table
              mate = mtf->second.at(male_mate_point);
            }

            // Compute the mate pose

            // compute twist between the two mate points
            KDL::Twist twist_err = diff(female_mate_pose, male_mate_pose);

            if(mate->joint->GetParent() && mate->joint->GetChild()) {
              if(false) {
                // detach joint
                mate->joint->Detach();
                // disable the mate
              }
            } else {
              if(false) {
                // attach joint
                mate->joint->Attach(female_atom->link, male_atom->link);
              }
            }
          }
        }
      }
    }
  }

  void AssemblySoup::instantiate_atom(const AtomModelPtr &atom_model, const sdf::Pose &pose)
  {
    AtomPtr atom = boost::make_shared<Atom>();

    // Fill out the link template with the instantiated information
    sdf::ElementPtr link_template(new sdf::Element());
    link_template->Copy(atom_model->link_template);

    gzwarn<<"instantiate: "<<link_template->ToString("> ")<<std::endl;

    link_template->GetAttribute("name")->Set(str(boost::format("%s_%0d") % atom_model->type % atom_id_counter_++));
    link_template->GetElement("pose")->GetValue()->Set(pose);

    // Create a new link owned by this model
    atom->link =
      model_->GetWorld()->GetPhysicsEngine()->CreateLink(
          boost::static_pointer_cast<gazebo::physics::Model>(model_));

    // Load the link information
    atom->link->Load(link_template);
    atom->link->Init();

    atoms_.push_back(atom);
  }

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

