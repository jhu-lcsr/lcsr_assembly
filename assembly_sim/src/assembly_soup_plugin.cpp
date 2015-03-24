#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

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
    sdf::ElementPtr mate_elem = _sdf->GetElement("mate");

    while(mate_elem)
    {
      // Create a new mate
      Mate mate;
      mate_elem->GetAttribute("type")->Get(mate.type);

      // Get the mate joint
      mate.joint_template->Copy(mate_elem->GetElement("joint"));

      // Get the mate symmetries
      sdf::ElementPtr symmetry_elem = _sdf->GetElement("symmetry");
      if(symmetry_elem) {
        sdf::ElementPtr rot_elem = _sdf->GetElement("rot");
        sdf::Vector3 rot_symmetry;
        rot_elem->GetValue()->Get(rot_symmetry);
      }

      // Store this atom
      mates_[mate.type] = mate;

      // Get the next atom element
      mate_elem = mate_elem->GetNextElement("mate");
    }

    // Get the description of the atoms in this soup
    sdf::ElementPtr atom_elem = _sdf->GetElement("atom");

    while(atom_elem)
    {
      // Create a new atom
      Atom atom;
      atom_elem->GetAttribute("type")->Get(atom.type);

      // Get the atom link
      atom.link_template->Copy(atom_elem->GetElement("link"));

      // Get the atom mates
      sdf::ElementPtr mate_elem = _sdf->GetElement("mate");
      while(mate_elem)
      {
        Mate mate;

        mate_elem->GetAttribute("type")->Get(mate.type);
        to_kdl(mate_elem->GetElement("pose"), mate.pose);

        std::string gender;
        mate_elem->GetAttribute("gender")->Get(gender);
        if(boost::iequals(gender, "female")) {
          atom.female_mates.push_back(mate);
          if(boost::iequals(gender, "male")) {
            atom.male_mates.push_back(mate);
          }
        }

        // Store this atom
        atoms_[atom.type] = atom;

        // Get the next atom element
        atom_elem = atom_elem->GetNextElement("atom");
      }
    }

    // Construct the initial assembly soup
    sdf::ElementPtr assembly_elem = _sdf->GetElement("assembly");
    sdf::ElementPtr assem_atom_elem = assembly_elem->GetElement("atom");

    while(assem_atom_elem)
    {
      // Get the instance details
      std::string type;
      assem_atom_elem->GetAttribute("type")->Get(type);
      sdf::Pose pose;
      assem_atom_elem->GetElement("pose")->GetValue()->Get(pose);

      // Instantiate the atom
      instantiate_atom(atoms_[type], pose);

      // Get the next atom element
      assem_atom_elem = assem_atom_elem->GetNextElement("atom");
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
            if(female_mate_point->model->type != male_mate_point->model->type) { continue; }

            // Get the mate between these two mate points
            MatePtr mate;
            mate_table_t::iterator mtf = mate_table_.find(female_mate_point);

            if(mtf == mate_table_.end())
            {
              // This female mate point needs to be added
              mate = boost::make_shared<Mate>();
              mate_point_map_t mate_point_map;
              mate_point_map.insert(male_mate_point, mate);
              mate_table_.insert(female_mate_point, mate_point_map);
            }
            else if(mtf->find(male_mate_point) == mtf->end())
            {
              // This male mate point needs to be added
              mate = boost::make_shared<Mate>();
              mtf->insert(male_mate_point, mate);
            }
            else
            {
              // This mate pair is already in the table
              mate = mtf->at(male_mate_point);
            }

            // Compute the mate pose
              

            // compute twist between the two mate points
            KDL::Twist twist_err = diff(female_mate_pose, male_mate_pose);

            if(/* mate exists */) {
              if(/* twist is too large */) {
                // detach joint
                mate->joint->Detach();
                // disable the mate
              }
            } else {
              if(/* twist is small enough */) {
                // attach joint
                mate->joint->Attach(female_atom->link, male_atom->link);
              }
            }
          }
        }
        // Check the proximity between the two mates
        KDL::Frame

      }
    }
  }

  void AssemblySoup::instantiate_atom(const Atom &atom, const sdf::Pose &pose)
  {
    // Fill out the link template with the instantiated information
    sdf::ElementPtr link_template(new sdf::Element());
    link_template->Copy(atom.link_template);
    link_template->GetAttribute("name")->Set(str(boost::format("%s_%0d") % atom.type % atom_id_counter_++));
    link_template->GetElement("pose")->GetValue()->Set(pose);

    // Create a new link owned by this model
    gazebo::physics::LinkPtr link =
      model_->GetWorld()->GetPhysicsEngine()->CreateLink(
          boost::static_pointer_cast<gazebo::physics::Model>(model_));

    // Load the link information
    link->Load(link_template);
    link->Init();
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

