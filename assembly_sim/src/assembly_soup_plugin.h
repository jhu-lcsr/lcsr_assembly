#ifndef __ASSEMBLY_SIM_ASSEMBLY_SOUP_PLUGIN_H
#define __ASSEMBLY_SIM_ASSEMBLY_SOUP_PLUGIN_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>


#include <kdl/frames.hpp>

namespace assembly_sim {
  struct MateModel;
  struct MatePointModel;
  struct AtomModel;

  struct Mate;
  struct MatePoint;
  struct Atom;

  typedef boost::shared_ptr<MateModel> MateModelPtr;
  typedef boost::shared_ptr<MatePointModel> MatePointModelPtr;
  typedef boost::shared_ptr<AtomModel> AtomModelPtr;

  typedef boost::shared_ptr<Mate> MatePtr;
  typedef boost::shared_ptr<MatePoint> MatePointPtr;
  typedef boost::shared_ptr<Atom> AtomPtr;

  // The model for a type of mate
  struct MateModel
  {
    std::string type;

    // Transforms from the base mate frame to alternative frames
    std::vector<KDL::Frame> symmetries;

    // The sdf template for the joint to be created
    sdf::ElementPtr joint_template;
  };

  struct MatePointModel
  {
    // The model used by this mate point
    MateModelPtr model;

    // The pose of the mate point in the owner frame
    KDL::Frame pose;
  };

  // The model for a type of atom
  struct AtomModel
  {
    // The type of atom
    std::string type;

    // Models for the mates
    std::vector<MatePointModelPtr> female_mate_points;
    std::vector<MatePointModelPtr> male_mate_points;

    // The sdf for the link to be created for this atom
    sdf::ElementPtr link_template;
  };

  // An instantiated mate
  struct Mate
  {
    // Joint associated with mate
    // If this is NULL then the mate is unoccupied
    gazebo::physics::JointPtr joint;
  };

  // A point where a mate can be created
  struct MatePoint
  {
    // The model used by this mate point
    MatePointModelPtr model;
  };

  // An instantiated atom
  struct Atom
  {
    // The model used by this atom
    AtomModelPtr model;

    // Mate points
    std::vector<MatePointPtr> female_mate_points;
    std::vector<MatePointPtr> male_mate_points;

    // The link on the assembly model
    gazebo::physics::LinkPtr link;
  };

  class AssemblySoup : public gazebo::ModelPlugin
  {
    public:
      AssemblySoup();
      void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void OnUpdate(const gazebo::common::UpdateInfo & /*_info*/);

      // Pointer to the model
    private:
      gazebo::physics::ModelPtr model_;

      // Pointer to the update event connection
      gazebo::event::ConnectionPtr updateConnection_;

    protected:
      size_t mate_id_counter_;
      size_t atom_id_counter_;

      std::map<std::string, MateModelPtr> mate_models_;
      std::map<std::string, AtomModelPtr> atom_models_;

      std::vector<AtomPtr> atoms_;

      typedef boost::unordered_map<MatePointPtr, MatePtr> mate_point_map_t;
      typedef boost::unordered_map<MatePointPtr, mate_point_map_t> mate_table_t;
      mate_table_t mate_table_;

      void instantiate_mate(const Mate &mate);
      void instantiate_atom(const AtomModelPtr &atom, const sdf::Pose &pose);
  };
}

#endif // ifndef __ASSEMBLY_SIM_ASSEMBLY_SOUP_PLUGIN_H
