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

  // The model for a type of mate
  struct MateModel
  {
    std::string type;

    // Transforms from the base mate frame to alternative frames
    std::vector<double> rot_symmetry;
    std::vector<kdl::Frame> symmetries;

    // The sdf template for the joint to be created
    sdf::ElementPtr joint_template;
  };
  typedef boost::shared_ptr<MateModel> MateModelPtr;

  // The model for a type of atom
  struct AtomModel
  {
    // The type of atom
    std::string type;

    // Models for the mates
    std::vector<MateModelPtr> female_mates;
    std::vector<MateModelPtr> male_mates;

    // The sdf for the link to be created for this atom
    sdf::ElementPtr link_template;
  };
  typedef boost::shared_ptr<AtomModel> AtomModel;

  // An instantiated mate
  struct Mate
  {
    // Joint associated with mate
    // If this is NULL then the mate is unoccupied
    gazebo::physics::JointPtr joint;
  };
  typedef boost::shared_ptr<Mate> MatePtr;

  // A point where a mate can be created
  struct MatePoint
  {
    // The model used by this mate point
    MateModelPtr model;
    // The pose of the mate point in the owner frame
    kdl::Frame pose;
  };
  typedef boost::shared_ptr<MatePoint> MatePointPtr;

  // An instantiated atom
  struct Atom
  {
    // The model used by this atom
    AtomModelPtr model;

    // Mate points
    std::Vector<MatePointPtr> female_mate_points;
    std::Vector<MatePointPtr> male_mate_points;

    // The link on the assembly model
    gazebo::physics::LinkPtr link;
  };
  typedef boost::shared_ptr<Atom> AtomPtr;

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

      std::map<std::string, Mate> mates_;
      std::map<std::string, Atom> atoms_;

      std::vector<gazebo::physics::LinkPtr> instantiated_atoms_;

      typedef boost::unordered_map<MatePointPtr, MatePtr> mate_point_map_t;
      typedef boost::unordered_map<MatePointPtr, mate_point_map_t> mate_table_t;
      mate_table_t mate_table_;

      void instantiate_mate(const Mate &mate);
      void instantiate_atom(const Atom &atom, const sdf::Pose &pose);

  };
}

#endif // ifndef __ASSEMBLY_SIM_ASSEMBLY_SOUP_PLUGIN_H
