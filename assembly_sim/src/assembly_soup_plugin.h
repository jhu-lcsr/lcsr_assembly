#ifndef __ASSEMBLY_SIM_ASSEMBLY_SOUP_PLUGIN_H
#define __ASSEMBLY_SIM_ASSEMBLY_SOUP_PLUGIN_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

#include <kdl/frames.hpp>

namespace assembly_sim {
  struct MateModel;
  struct AtomModel;

  struct Mate;
  struct MatePoint;
  struct Atom;

  typedef boost::shared_ptr<MateModel> MateModelPtr;
  typedef boost::shared_ptr<AtomModel> AtomModelPtr;

  typedef boost::shared_ptr<Mate> MatePtr;
  typedef boost::shared_ptr<MatePoint> MatePointPtr;
  typedef boost::shared_ptr<Atom> AtomPtr;

  // The model for how a mate behaves
  struct MateModel
  {
    enum State {
      NONE = 0, // No state / undefined
      UNMATED = 1, // There is no interaction between this mate's atoms
      MATING = 2, // This mate's atoms are connected by the transitional mechanism
      MATED = 3 // This mate's atoms are connected by a static joint
    };

    MateModel(std::string type_) : type(type_) {}

    std::string type;

    // Transforms from the base mate frame to alternative frames
    std::vector<KDL::Frame> symmetries;

    // The sdf template for the joint to be created
    boost::shared_ptr<sdf::SDF> joint_template_sdf;
    sdf::ElementPtr joint_template;

    // Load parameters from SDF
    virtual void load(sdf::ElementPtr mate_elem) = 0;

    // Update mates to attach / detach based on atom state (asynchronous with gazebo)
    virtual State getStateUpdate(const MatePtr mate) = 0;

    // Update mate state (synchronous with gazebo)
    virtual void updateState(MatePtr mate) = 0;
  };

  struct MatePoint
  {
    // The mate model used by this mate point
    MateModelPtr model;
    // Mate point index
    size_t id;
    // The pose of the mate point in the owner frame
    KDL::Frame pose;
  };

  // An instantiated mate
  struct Mate
  {
    Mate(
      gazebo::physics::ModelPtr gazebo_model,
      MatePointPtr female_mate_point_,
      MatePointPtr male_mate_point_,
      AtomPtr female_atom,
      AtomPtr male_atom);

    // Mate model (same as mate points)
    MateModelPtr model;

    // Attachment state
    MateModel::State state, pending_state;

    // Joint SDF
    sdf::ElementPtr joint_sdf;
    // Joint associated with mate
    // If this is NULL then the mate is unoccupied
    gazebo::physics::JointPtr joint;

    // Max erp
    double max_stop_erp;
    double max_erp;

    // Atoms associated with this mate
    AtomPtr female;
    AtomPtr male;

    // Mate points
    MatePointPtr female_mate_point;
    MatePointPtr male_mate_point;

    // The pose of the joint anchor point relative to the mate point.
    // This gets set each time two atoms are mated, and enables joints
    // to be consistently strong.
    KDL::Frame anchor_offset;
    KDL::Twist mate_error;
  };

  // The model for a type of atom
  struct AtomModel
  {
    // The type of atom
    std::string type;

    // Models for the mates
    std::vector<MatePointPtr> female_mate_points;
    std::vector<MatePointPtr> male_mate_points;

    // The sdf for the link to be created for this atom
    boost::shared_ptr<sdf::SDF> link_template_sdf;
    sdf::ElementPtr link_template;
  };

  // An instantiated atom
  struct Atom
  {
    // The model used by this atom
    AtomModelPtr model;

    // The link on the assembly model
    gazebo::physics::LinkPtr link;
  };

  class AssemblySoup : public gazebo::ModelPlugin
  {
    public:
      AssemblySoup();
      void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void OnUpdate(const gazebo::common::UpdateInfo & /*_info*/);
      ~AssemblySoup();

      // Pointer to the model
    private:
      gazebo::physics::ModelPtr model_;
      sdf::ElementPtr sdf_;

      // Pointer to the update event connection
      gazebo::event::ConnectionPtr updateConnection_;

      ros::Publisher male_mate_pub_;
      ros::Publisher female_mate_pub_;

      // for broadcasting lists of mates
      bool publish_active_mates_;
      ros::Publisher active_mates_pub_;

      // used to synchronize main update thread with check thread
      boost::mutex update_mutex_;

      // update thread
      boost::thread state_update_thread_;
      void stateUpdateLoop();
      void getStateUpdates();
      bool running_;

    protected:
      size_t mate_id_counter_;
      size_t atom_id_counter_;

      double max_trans_err_;
      double max_rot_err_;

      std::map<std::string, MateModelPtr> mate_models_;
      std::map<std::string, AtomModelPtr> atom_models_;

      std::vector<AtomPtr> atoms_;

      // all mates
      boost::unordered_set<MatePtr> mates_;

      // mates to attach/detach in OnUpdate thread
      boost::unordered_set<MatePtr> mates_to_update_;

      typedef boost::unordered_map<MatePointPtr, MatePtr> mate_point_map_t;
      typedef boost::unordered_map<MatePointPtr, mate_point_map_t> mate_table_t;
      mate_table_t mate_table_;

      // for broadcasting coordinate transforms
      bool broadcast_tf_;
      std::string tf_world_frame_;

      clock_t last_tick_;
      int updates_per_second_;

  };
}

#endif // ifndef __ASSEMBLY_SIM_ASSEMBLY_SOUP_PLUGIN_H
