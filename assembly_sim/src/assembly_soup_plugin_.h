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
    std::string type;

    // Transforms from the base mate frame to alternative frames
    std::vector<KDL::Frame> symmetries;

    // The sdf template for the joint to be created
    boost::shared_ptr<sdf::SDF> joint_template_sdf;
    sdf::ElementPtr joint_template;

    // Load parameters from SDF
    virtual void load(sdf::ElementPtr mate_elem) = 0;

    // Update mates to attach / detach based on atom state
    virtual void update(
        const MatePtr mate
        std::vector<MatePtr> &mates_to_detach,
        std::vector<MatePtr> &mates_to_attach) = 0;
  };

  struct ProximityMateModel : public MateModel
  {
    // Threshold for attaching a mate
    double attach_threshold_linear;
    double attach_threshold_angular;

    // Threshold for detaching a mate
    double detach_threshold_linear;
    double detach_threshold_angular;

    virtual void load(sdf::ElementPtr mate_elem)
    {
      // Get the attach/detach thresholds
      sdf::ElementPtr attach_threshold_elem = mate_elem->GetElement("attach_threshold");
      if(attach_threshold_elem and attach_threshold_elem->HasElement("linear") and attach_threshold_elem->HasElement("angular"))
      {
        attach_threshold_elem->GetElement("linear")->GetValue()->Get(attach_threshold_linear);
        attach_threshold_elem->GetElement("angular")->GetValue()->Get(attach_threshold_angular);

        gzwarn<<(boost::format("Attach threshold linear: %f angular %f") % attach_threshold_linear % mate_model->attach_threshold_angular)<<std::endl;
      } else {
        gzerr<<"No attach_threshold / linear / angular elements!"<<std::endl;
      }

      sdf::ElementPtr detach_threshold_elem = mate_elem->GetElement("detach_threshold");
      if(detach_threshold_elem and detach_threshold_elem->HasElement("linear") and detach_threshold_elem->HasElement("angular"))
      {
        detach_threshold_elem->GetElement("linear")->GetValue()->Get(detach_threshold_linear);
        detach_threshold_elem->GetElement("angular")->GetValue()->Get(detach_threshold_angular);

        gzwarn<<(boost::format("Detach threshold linear: %f angular %f") % attach_threshold_linear % mate_model->attach_threshold_angular)<<std::endl;
      } else {
        gzerr<<"No detach_threshold / linear / angular elements!"<<std::endl;
      }
    }

    virtual void update(
        const MatePtr mate
        std::vector<MatePtr> &mates_to_detach,
        std::vector<MatePtr> &mates_to_affect,
        std::vector<MatePtr> &mates_to_attach)
    {
      // Convenient references
      AtomPtr &female_atom = mate->female;
      AtomPtr &male_atom = mate->male;

      MatePointPtr &female_mate_point = mate->female_mate_point;
      MatePointPtr &male_mate_point = mate->male_mate_point;

      KDL::Frame female_atom_frame;
      to_kdl(female_atom->link->GetWorldPose(), female_atom_frame);
      KDL::Frame female_mate_frame = female_atom_frame * female_mate_point->pose;

      KDL::Frame male_atom_frame;
      to_kdl(male_atom->link->GetWorldPose(), male_atom_frame);
      // Compute the world pose of the male mate frame
      // This takes into account the attachment displacement (anchor_offset)
      KDL::Frame male_mate_frame = male_atom_frame * male_mate_point->pose * mate->anchor_offset;

      // compute twist between the two mate points
      KDL::Twist twist_err = diff(female_mate_frame, male_mate_frame);

      // Only analyze mates with associated joints
      if(mate->joint) {
        //if(female_mate_point->id == 0) gzwarn<<" --- err linear: "<<twist_err.vel.Norm()<<" angular: "<<twist_err.rot.Norm()<<std::endl;

        // Determine if mated atoms need to be detached
        if(mate->joint->GetParent() and mate->joint->GetChild()) {
          //gzwarn<<"Parts are mated!"<<std::endl;

          if(twist_err.vel.Norm() > mate_model->detach_threshold_linear or
             twist_err.rot.Norm() > mate_model->detach_threshold_angular)
          {
            mates_to_detach.push_back(mate);

          } else if(publish_active_mates_) {
            mates_msg.female.push_back(mate->joint->GetParent()->GetName());
            mates_msg.male.push_back(mate->joint->GetChild()->GetName());
          }

        } else {
          //gzwarn<<"Parts are not mated!"<<std::endl;
          if(twist_err.vel.Norm() < mate_model->attach_threshold_linear and
             twist_err.rot.Norm() < mate_model->attach_threshold_angular)
          {
            mates_to_attach.push_back(mate);
          }
        }

      } else {
        //gzwarn<<"No joint for mate from "<<female_atom->link->GetName()<<" -> "<<male_atom->link->GetName()<<std::endl;
      }
    }

    virtual void attach(MatePtr mate)
    {
      // Get the male atom frame
      KDL::Frame male_atom_frame;
      to_kdl(mate->male->link->GetWorldPose(), male_atom_frame);

      // attach two atoms via joint
      mate->joint->Attach(mate->female->link, mate->male->link);

      // get the location of the joint in the child (male atom) frame, as specified by the SDF
      KDL::Frame initial_anchor_frame, actual_anchor_frame;
      to_kdl(mate->joint->GetInitialAnchorPose(), initial_anchor_frame);
      actual_anchor_frame = male_atom_frame*initial_anchor_frame;

      // Set the anchor position (location of the joint)
      // This is in the WORLD frame
      // IMPORTANT: This avoids injecting energy into the system in the form of a constraint violation
      gazebo::math::Pose actual_anchor_pose;
      to_gazebo(actual_anchor_frame, actual_anchor_pose);
      mate->joint->SetAnchor(0, actual_anchor_pose.pos);

      // Save the anchor offset (mate point to anchor)
      mate->anchor_offset = (
          actual_anchor_frame.Inverse() *   // anchor to world
          male_atom_frame *                 // world to atom
          mate->male_mate_point->pose // atom to mate point
          ).Inverse();

      gzwarn<<" --- joint pose: "<<std::endl<<initial_anchor_frame<<std::endl;
    }

    virtual void detach(MatePtr mate)
    {
      // Simply detach joint
      mate->joint->Detach();
    }
  };

  struct DipoleMateModel : public MateModel
  {

    double min_force_linear, min_force_angular, min_force_linear_deadband, min_force_angular_deadband;
    double max_force_linear, max_force_angular, max_force_linear_deadband, max_force_angular_deadband;
    double moment;

    virtual void load(sdf::ElementPtr mate_elem)
    {
      // Get the attach/detach thresholds
      sdf::ElementPtr force_elem = mate_elem->GetElement("force");
      if(force_alam and force_alem->HasElement("min") and force_elem->HasElement("max")) {

        sdf::ElementPtr min_elem = force_elem->GetElement("min");
        sdf::ElementPtr max_elem = force_elem->GetElement("max");

        sdf::ElementPtr min_linear_elem = min_elem->GetElement("linear");
        sdf::ElementPtr min_angular_elem = min_elem->GetElement("angular");

        sdf::ElementPtr max_linear_elem = max_elem->GetElement("linear");
        sdf::ElementPtr max_angular_elem = max_elem->GetElement("angular");

        min_linear_elem->GetAttribute("threshold")->Get(min_force_linear);
        min_angular_elem->GetAttribute("threshold")->Get(min_force_angular);
        max_linear_elem->GetAttribute("threshold")->Get(max_force_linear);
        max_angular_elem->GetAttribute("threshold")->Get(max_force_angular);

        min_linear_elem->GetAttribute("deadband")->Get(min_force_linear_deadband);
        min_angular_elem->GetAttribute("deadband")->Get(min_force_angular_deadband);
        max_linear_elem->GetAttribute("deadband")->Get(max_force_linear_deadband);
        max_angular_elem->GetAttribute("deadband")->Get(max_force_angular_deadband);
      } else {
        gzerr<<"No force threshold elements!"<<std::endl;
      }

      // Get the dipole moments (along Z axis)
      sdf::ElementPtr moment_elem = mate_elem->GetElement("moment");
      moment_elem->Get(moment);
    }

    virtual void update(
        const MatePtr mate
        std::vector<MatePtr> &mates_to_detach,
        std::vector<MatePtr> &mates_to_attach)
    {
      // Convenient references
      AtomPtr &female_atom = mate->female;
      AtomPtr &male_atom = mate->male;

      MatePointPtr &female_mate_point = mate->female_mate_point;
      MatePointPtr &male_mate_point = mate->male_mate_point;

      KDL::Frame female_atom_frame;
      to_kdl(female_atom->link->GetWorldPose(), female_atom_frame);
      KDL::Frame female_mate_frame = female_atom_frame * female_mate_point->pose;

      KDL::Frame male_atom_frame;
      to_kdl(male_atom->link->GetWorldPose(), male_atom_frame);
      // Compute the world pose of the male mate frame
      // This takes into account the attachment displacement (anchor_offset)
      KDL::Frame male_mate_frame = male_atom_frame * male_mate_point->pose * mate->anchor_offset;

      // compute twist between the two mate points
      KDL::Twist twist_err = diff(female_mate_frame, male_mate_frame);

      // Only analyze mates with associated joints
      if(mate->joint) {
        //if(female_mate_point->id == 0) gzwarn<<" --- err linear: "<<twist_err.vel.Norm()<<" angular: "<<twist_err.rot.Norm()<<std::endl;

        // Determine if mated atoms need to be detached
        if(mate->joint->GetParent() and mate->joint->GetChild()) {
          //gzwarn<<"Parts are mated!"<<std::endl;

          if(twist_err.vel.Norm() > mate_model->detach_threshold_linear or
             twist_err.rot.Norm() > mate_model->detach_threshold_angular)
          {
            mates_to_detach.push_back(mate);

          } else if(publish_active_mates_) {
            mates_msg.female.push_back(mate->joint->GetParent()->GetName());
            mates_msg.male.push_back(mate->joint->GetChild()->GetName());
          }

        } else {
          //gzwarn<<"Parts are not mated!"<<std::endl;
          if(twist_err.vel.Norm() < mate_model->attach_threshold_linear and
             twist_err.rot.Norm() < mate_model->attach_threshold_angular)
          {
            mates_to_attach.push_back(mate);
          }
        }

      } else {
        //gzwarn<<"No joint for mate from "<<female_atom->link->GetName()<<" -> "<<male_atom->link->GetName()<<std::endl;
      }
    }

    virtual void attach(
        MatePtr mate)
    {

    }
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

  // An instantiated mate
  struct Mate
  {
    typedef enum {
      UNMATED = 0, // There is no interaction between this mate's atoms
      MATING = 1, // This mate's atoms are connected by the transitional mechanism
      MATED = 2 // This mate's atoms are connected by a static joint
    } MateState;

    // TODO add constructor that does lines 300-326 from the cpp file
    Mate(
      gazebo::physics::ModelPtr gazebo_model,
      MatePointPtr female_mate_point_,
      MatePointPtr male_mate_point_,
      AtomPtr female_atom,
      AtomPtr male_atom);

    // Attachment state
    MateState state;

    // Joint SDF
    sdf::ElementPtr joint_sdf;
    // Joint associated with mate
    // If this is NULL then the mate is unoccupied
    gazebo::physics::JointPtr joint;

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
      boost::thread check_thread_;
      void CheckProximityLoop();
      void DoProximityCheck(); // run one collision check
      bool running_;

      // mates to attach/detach in OnUpdate thread
      std::vector<MatePtr> mates_to_attach;
      std::vector<MatePtr> mates_to_detach;

    protected:
      size_t mate_id_counter_;
      size_t atom_id_counter_;

      double max_trans_err_;
      double max_rot_err_;

      std::map<std::string, MateModelPtr> mate_models_;
      std::map<std::string, AtomModelPtr> atom_models_;

      std::vector<AtomPtr> atoms_;

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
