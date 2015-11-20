#ifndef __LCSR_ASSEMBLY_ASSEMBLY_SIM_MODELS_H__
#define __LCSR_ASSEMBLY_ASSEMBLY_SIM_MODELS_H__

#include "util.h"

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
    MateModel(std::string type_) : type(type_) {}

    std::string type;

    // Transforms from the base mate frame to alternative frames
    std::vector<KDL::Frame> symmetries;

    // The mate sdf parameters
    sdf::ElementPtr mate_elem;

    // The sdf template for the joint to be created
    boost::shared_ptr<sdf::SDF> joint_template_sdf;
    sdf::ElementPtr joint_template;

    virtual MatePtr createMate(
        gazebo::physics::ModelPtr gazebo_model,
        MatePointPtr female_mate_point,
        MatePointPtr male_mate_point,
        AtomPtr female_atom,
        AtomPtr male_atom) = 0;
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
    enum State {
      NONE = 0, // No state / undefined
      UNMATED = 1, // There is no interaction between this mate's atoms
      MATING = 2, // This mate's atoms are connected by the transitional mechanism
      MATED = 3 // This mate's atoms are connected by a static joint
    };

    Mate(
      gazebo::physics::ModelPtr gazebo_model,
      MatePointPtr female_mate_point_,
      MatePointPtr male_mate_point_,
      AtomPtr female_atom,
      AtomPtr male_atom);

    // Update mates to attach / detach based on atom state (asynchronous with gazebo)
    // This might store 
    virtual State getNewState() = 0;

    // Update mate state (synchronous with gazebo)
    virtual void setState(State new_state) = 0;

    // Update calculations needed to be done every tick
    virtual void update() = 0;

    // Mate model (same as mate points)
    MateModelPtr model;

    // Attachment state
    Mate::State state;

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

    // Max erp
    double max_stop_erp;
    double max_erp;

    // The pose of the joint anchor point relative to the mate point.
    // This gets set each time two atoms are mated, and enables joints
    // to be consistently strong.
    KDL::Frame anchor_offset;
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

  struct ProximityMate : public Mate
  {
    ProximityMate(
        gazebo::physics::ModelPtr gazebo_model,
        MatePointPtr female_mate_point_,
        MatePointPtr male_mate_point_,
        AtomPtr female_atom,
        AtomPtr male_atom) :
      Mate(gazebo_model, female_mate_point_, male_mate_point_, female_atom, male_atom)
    {
      this->load();
    }

    // Threshold for attaching a mate
    double attach_threshold_linear;
    double attach_threshold_angular;

    // Threshold for detaching a mate
    double detach_threshold_linear;
    double detach_threshold_angular;

    KDL::Twist mate_error;

    void load()
    {
      sdf::ElementPtr mate_elem = model->mate_elem;

      // Get the attach/detach thresholds
      sdf::ElementPtr attach_threshold_elem = mate_elem->GetElement("attach_threshold");
      if(attach_threshold_elem and attach_threshold_elem->HasElement("linear") and attach_threshold_elem->HasElement("angular"))
      {
        attach_threshold_elem->GetElement("linear")->GetValue()->Get(attach_threshold_linear);
        attach_threshold_elem->GetElement("angular")->GetValue()->Get(attach_threshold_angular);

        gzwarn<<(boost::format("Attach threshold linear: %f angular %f") % attach_threshold_linear % attach_threshold_angular)<<std::endl;
      } else {
        gzerr<<"No attach_threshold / linear / angular elements!"<<std::endl;
      }

      sdf::ElementPtr detach_threshold_elem = mate_elem->GetElement("detach_threshold");
      if(detach_threshold_elem and detach_threshold_elem->HasElement("linear") and detach_threshold_elem->HasElement("angular"))
      {
        detach_threshold_elem->GetElement("linear")->GetValue()->Get(detach_threshold_linear);
        detach_threshold_elem->GetElement("angular")->GetValue()->Get(detach_threshold_angular);

        gzwarn<<(boost::format("Detach threshold linear: %f angular %f") % attach_threshold_linear % attach_threshold_angular)<<std::endl;
      } else {
        gzerr<<"No detach_threshold / linear / angular elements!"<<std::endl;
      }
    }

    virtual Mate::State getNewState()
    {
      Mate::State new_state = Mate::NONE;

      // Convenient references
      AtomPtr &female_atom = this->female;
      AtomPtr &male_atom = this->male;

      MatePointPtr &female_mate_point = this->female_mate_point;
      MatePointPtr &male_mate_point = this->male_mate_point;

      KDL::Frame female_atom_frame;
      to_kdl(female_atom->link->GetWorldPose(), female_atom_frame);
      KDL::Frame female_mate_frame = female_atom_frame * female_mate_point->pose;

      KDL::Frame male_atom_frame;
      to_kdl(male_atom->link->GetWorldPose(), male_atom_frame);

      // Compute the world pose of the male mate frame
      // This takes into account the attachment displacement (anchor_offset)
      KDL::Frame male_mate_frame = male_atom_frame * male_mate_point->pose * this->anchor_offset;

      // compute twist between the two mate points
      KDL::Twist twist_err = diff(female_mate_frame, male_mate_frame);

      // Only analyze mates with associated joints
      if(this->joint) {
        //if(female_mate_point->model->id == 0) gzwarn<<" --- err linear: "<<twist_err.vel.Norm()<<" angular: "<<twist_err.rot.Norm()<<std::endl;

        // Determine if mated atoms need to be detached
        if(this->joint->GetParent() and this->joint->GetChild()) {
          //gzwarn<<"Parts are mated!"<<std::endl;
          //gzwarn<<twist_err.vel.Norm()<<", "<<twist_err.rot.Norm()<<" VS "<<this->mate_error.vel.Norm()<<", "<<this->mate_error.rot.Norm()<<std::endl;

          if(twist_err.vel.Norm() > detach_threshold_linear or
             twist_err.rot.Norm() > detach_threshold_angular)
          {
            new_state = Mate::UNMATED;
          } else if(
              twist_err.vel.Norm() <= this->mate_error.vel.Norm() and
              twist_err.rot.Norm() <= this->mate_error.rot.Norm() + 1E-4)
          {
            // re-mate but closer to the desired point
            this->mate_error = twist_err;
            new_state = Mate::MATED;
          }

        } else {
          //gzwarn<<"Parts are not mated!"<<std::endl;
          if(twist_err.vel.Norm() < attach_threshold_linear and
             twist_err.rot.Norm() < attach_threshold_angular)
          {
            this->mate_error = twist_err;
            new_state = Mate::MATED;
          }
        }

      } else {
        gzwarn<<"No joint for mate from "<<female_atom->link->GetName()<<" -> "<<male_atom->link->GetName()<<std::endl;
      }

      return new_state;
    }

    virtual void setState(Mate::State pending_state)
    {
      switch(pending_state) {
        case Mate::NONE:
          break;

        case Mate::UNMATED:
          if(this->state == Mate::MATED) {
            gzwarn<<"Detaching "<<this->female->link->GetName()<<" from "<<this->male->link->GetName()<<"!"<<std::endl;
            this->detach();
          }
          break;

        case Mate::MATING:
          gzwarn<<"ProximityMate does not support Mate::MATING"<<std::endl;
          break;

        case Mate::MATED:
          if(this->state != Mate::MATED) {
            gzwarn<<"Attaching "<<this->female->link->GetName()<<" to "<<this->male->link->GetName()<<"!"<<std::endl;
          } else {
            gzwarn<<"Reattaching "<<this->female->link->GetName()<<" to "<<this->male->link->GetName()<<"!"<<std::endl;
          }
          this->attach();
          break;
      };
    }

    virtual void update()
    {
    }

  private:
    void attach()
    {
      // Get the male atom frame
      KDL::Frame male_atom_frame;
      to_kdl(this->male->link->GetWorldPose(), male_atom_frame);

      // detach the atoms if they're already attached (they're going to be re-attached)
      this->detach();

      // attach two atoms via joint
      this->joint->Attach(this->female->link, this->male->link);

      // set stiffness based on proximity to goal
      double lin_err = this->mate_error.vel.Norm();
      double ang_err = this->mate_error.rot.Norm();
      double max_lin_err = detach_threshold_linear;
      double max_ang_err = detach_threshold_angular;
      //this->joint->SetAttribute(
      //"stop_erp", 0,
      //this->max_stop_erp *
      //std::min(std::max(0.0, max_lin_err - lin_err), max_lin_err) / max_lin_err *
      //std::min(std::max(0.0, max_ang_err - ang_err), max_ang_err) / max_ang_err);
      //this->joint->SetAttribute(
      //"erp", 0,
      //this->max_erp *
      //std::min(std::max(0.0, max_lin_err - lin_err), max_lin_err) / max_lin_err *
      //std::min(std::max(0.0, max_ang_err - ang_err), max_ang_err) / max_ang_err);

      //this->joint->SetHighStop(0, lin_err);

      // get the location of the joint in the child (male atom) frame, as specified by the SDF
      KDL::Frame initial_anchor_frame, actual_anchor_frame;
      to_kdl(this->joint->GetInitialAnchorPose(), initial_anchor_frame);
      actual_anchor_frame = male_atom_frame*initial_anchor_frame;

      // Set the anchor position (location of the joint)
      // This is in the WORLD frame
      // IMPORTANT: This avoids injecting energy into the system in the form of a constraint violation
      gazebo::math::Pose actual_anchor_pose;
      to_gazebo(actual_anchor_frame, actual_anchor_pose);
      this->joint->SetAnchor(0, actual_anchor_pose.pos);

      // Save the anchor offset (mate point to anchor)
      this->anchor_offset = (
          actual_anchor_frame.Inverse() *   // anchor to world
          male_atom_frame *                 // world to atom
          this->male_mate_point->pose // atom to mate point
          ).Inverse();

      //gzwarn<<" ---- initial anchor pose: "<<std::endl<<initial_anchor_frame<<std::endl;
      //gzwarn<<" ---- actual anchor pose: "<<std::endl<<actual_anchor_frame<<std::endl;
      gzwarn<<" ---- mate error: "<<this->mate_error.vel.Norm()<<", "<<this->mate_error.rot.Norm()<<std::endl;

      this->state = Mate::MATED;
    }

    void detach()
    {
      // Simply detach joint
      this->joint->Detach();

      this->state = Mate::UNMATED;
    }
  };

  struct ProximityMateModel : public MateModel
  {
    ProximityMateModel(std::string type) : MateModel(type) {}

    virtual MatePtr createMate(
        gazebo::physics::ModelPtr gazebo_model,
        MatePointPtr female_mate_point,
        MatePointPtr male_mate_point,
        AtomPtr female_atom,
        AtomPtr male_atom)
    {
      return boost::make_shared<ProximityMate>(
          gazebo_model,
          female_mate_point,
          male_mate_point,
          female_atom,
          male_atom);
    }
  };


#if 1
  struct DipoleMate : public ProximityMate
  {
    DipoleMate(
        gazebo::physics::ModelPtr gazebo_model,
        MatePointPtr female_mate_point_,
        MatePointPtr male_mate_point_,
        AtomPtr female_atom,
        AtomPtr male_atom) :
      ProximityMate(gazebo_model, female_mate_point_, male_mate_point_, female_atom, male_atom)
    {
      this->load();
    }
    virtual void update()
    {

    }
  };

  struct DipoleMateModel : public ProximityMateModel
  {
    DipoleMateModel(std::string type) : ProximityMateModel(type) {}

    virtual MatePtr createMate(
      gazebo::physics::ModelPtr gazebo_model,
      MatePointPtr female_mate_point,
      MatePointPtr male_mate_point,
      AtomPtr female_atom,
      AtomPtr male_atom)
    {
      return boost::make_shared<DipoleMate>(
          gazebo_model,
          female_mate_point,
          male_mate_point,
          female_atom,
          male_atom);
    }
  };

#else
  struct DipoleMateModel : public MateModel
  {
    DipoleMateModel(std::string type) : MateModel(type) {}

    double min_force_linear, min_force_angular, min_force_linear_deadband, min_force_angular_deadband;
    double max_force_linear, max_force_angular, max_force_linear_deadband, max_force_angular_deadband;
    double moment;

    virtual void load(sdf::ElementPtr mate_elem)
    {
      // Get the attach/detach thresholds
      sdf::ElementPtr force_elem = mate_elem->GetElement("force");
      if(force_elem and force_elem->HasElement("min") and force_elem->HasElement("max")) {

        sdf::ElementPtr min_elem = force_elem->GetElement("min");
        sdf::ElementPtr max_elem = force_elem->GetElement("max");

        sdf::ElementPtr min_linear_elem = min_elem->GetElement("linear");
        sdf::ElementPtr min_angular_elem = min_elem->GetElement("angular");

        sdf::ElementPtr max_linear_elem = max_elem->GetElement("linear");
        sdf::ElementPtr max_angular_elem = max_elem->GetElement("angular");

        min_linear_elem->GetAttribute("threshold")->Get(min_force_linear);
        max_linear_elem->GetAttribute("threshold")->Get(max_force_linear);
        min_angular_elem->GetAttribute("threshold")->Get(min_force_angular);
        max_angular_elem->GetAttribute("threshold")->Get(max_force_angular);

        min_linear_elem->GetAttribute("deadband")->Get(min_force_linear_deadband);
        max_linear_elem->GetAttribute("deadband")->Get(max_force_linear_deadband);
        min_angular_elem->GetAttribute("deadband")->Get(min_force_angular_deadband);
        max_angular_elem->GetAttribute("deadband")->Get(max_force_angular_deadband);
      } else {
        gzerr<<"No force threshold elements!"<<std::endl;
      }

      // Get the dipole moments (along Z axis)
      sdf::ElementPtr moment_elem = mate_elem->GetElement("moment");
      moment_elem->Get(moment);
    }

    virtual Mate::State getNewState(const MatePtr mate)
    {
      Mate::State new_state = Mate::NONE;

      // Convenient references
      AtomPtr &female_atom = this->female;
      AtomPtr &male_atom = this->male;

      MatePointPtr &female_mate_point = this->female_mate_point;
      MatePointPtr &male_mate_point = this->male_mate_point;

      KDL::Frame female_atom_frame;
      to_kdl(female_atom->link->GetWorldPose(), female_atom_frame);
      KDL::Frame female_mate_frame = female_atom_frame * female_mate_point->pose;

      KDL::Frame male_atom_frame;
      to_kdl(male_atom->link->GetWorldPose(), male_atom_frame);

      // Compute the world pose of the male mate frame
      // This takes into account the attachment displacement (anchor_offset)
      KDL::Frame male_mate_frame = male_atom_frame * male_mate_point->pose * this->anchor_offset;

      // compute twist between the two mate points
      KDL::Twist twist_err = diff(female_mate_frame, male_mate_frame);

      // Only analyze mates with associated joints
      if(this->joint) {
        //if(female_mate_point->model->id == 0) gzwarn<<" --- err linear: "<<twist_err.vel.Norm()<<" angular: "<<twist_err.rot.Norm()<<std::endl;

        // Determine if mated atoms need to be detached
        if(this->joint->GetParent() and this->joint->GetChild()) {
          //gzwarn<<"Parts are mated!"<<std::endl;

          if(twist_err.vel.Norm() > detach_threshold_linear or
             twist_err.rot.Norm() > detach_threshold_angular)
          {
            new_state = Mate::UNMATED;
          } else if(
              twist_err.vel.Norm() < this->mate_error.vel.Norm() and
              twist_err.rot.Norm() < this->mate_error.rot.Norm())
          {
            // re-mate but closer to the desired point
            this->mate_error = twist_err;
            new_state = Mate::MATED;
          }

        } else {
          //gzwarn<<"Parts are not mated!"<<std::endl;
          if(twist_err.vel.Norm() < attach_threshold_linear and
             twist_err.rot.Norm() < attach_threshold_angular)
          {
            this->mate_error = twist_err;
            new_state = Mate::MATED;
          }
        }

      } else {
        gzwarn<<"No joint for mate from "<<female_atom->link->GetName()<<" -> "<<male_atom->link->GetName()<<std::endl;
      }

      return new_state;
    }

    virtual void setState(MatePtr mate, Mate::State pending_state)
    {
      switch(pending_state) {
        case Mate::NONE:
          break;

        case Mate::UNMATED:
          if(this->state == Mate::MATED) {
            gzwarn<<"Detaching "<<this->female->link->GetName()<<" from "<<this->male->link->GetName()<<"!"<<std::endl;
            this->detach(mate);
          }
          break;

        case Mate::MATING:
          gzwarn<<"Attracting "<<this->female->link->GetName()<<" to "<<this->male->link->GetName()<<"!"<<std::endl;
          if(this->state != Mate::MATED) {
          } else {
          }
          break;

        case Mate::MATED:
          if(this->state != Mate::MATED) {
            gzwarn<<"Attaching "<<this->female->link->GetName()<<" to "<<this->male->link->GetName()<<"!"<<std::endl;
          } else {
            gzwarn<<"Reattaching "<<this->female->link->GetName()<<" to "<<this->male->link->GetName()<<"!"<<std::endl;
          }
          this->attach(mate);
          break;
      };

      this->pending_state = Mate::NONE;
    }

    virtual void update(MatePtr mate)
    {
      // Compute new force
      // TODO

      switch(this->state) {
        case Mate::NONE:
          break;

        case Mate::UNMATED:
          break;

        case Mate::MATING:
          break;

        case Mate::MATED:
          break;
      };
    }

    void attach(MatePtr mate)
    {
      // Get the male atom frame
      KDL::Frame male_atom_frame;
      to_kdl(this->male->link->GetWorldPose(), male_atom_frame);

      // detach the atoms if they're already attached (they're going to be re-attached)
      this->detach(mate);

      // attach two atoms via joint
      this->joint->Attach(this->female->link, this->male->link);

      // set stiffness based on proximity to goal
      double lin_err = this->mate_error.vel.Norm();
      double ang_err = this->mate_error.rot.Norm();
      double max_lin_err = detach_threshold_linear;
      double max_ang_err = detach_threshold_angular;
      //this->joint->SetAttribute(
      //"stop_erp", 0,
      //this->max_stop_erp *
      //std::min(std::max(0.0, max_lin_err - lin_err), max_lin_err) / max_lin_err *
      //std::min(std::max(0.0, max_ang_err - ang_err), max_ang_err) / max_ang_err);
      //this->joint->SetAttribute(
      //"erp", 0,
      //this->max_erp *
      //std::min(std::max(0.0, max_lin_err - lin_err), max_lin_err) / max_lin_err *
      //std::min(std::max(0.0, max_ang_err - ang_err), max_ang_err) / max_ang_err);

      //this->joint->SetHighStop(0, lin_err);

      // get the location of the joint in the child (male atom) frame, as specified by the SDF
      KDL::Frame initial_anchor_frame, actual_anchor_frame;
      to_kdl(this->joint->GetInitialAnchorPose(), initial_anchor_frame);
      actual_anchor_frame = male_atom_frame*initial_anchor_frame;

      // Set the anchor position (location of the joint)
      // This is in the WORLD frame
      // IMPORTANT: This avoids injecting energy into the system in the form of a constraint violation
      gazebo::math::Pose actual_anchor_pose;
      to_gazebo(actual_anchor_frame, actual_anchor_pose);
      this->joint->SetAnchor(0, actual_anchor_pose.pos);

      // Save the anchor offset (mate point to anchor)
      this->anchor_offset = (
          actual_anchor_frame.Inverse() *   // anchor to world
          male_atom_frame *                 // world to atom
          this->male_mate_point->pose // atom to mate point
          ).Inverse();

      //gzwarn<<" ---- initial anchor pose: "<<std::endl<<initial_anchor_frame<<std::endl;
      //gzwarn<<" ---- actual anchor pose: "<<std::endl<<actual_anchor_frame<<std::endl;
      gzwarn<<" ---- mate error: "<<this->mate_error.vel.Norm()<<", "<<this->mate_error.rot.Norm()<<std::endl;

      this->state = Mate::MATED;
    }

    void detach(MatePtr mate)
    {
      // Simply detach joint
      this->joint->Detach();

      this->state = Mate::UNMATED;
    }
  };
#endif

}

#endif // ifndef __LCSR_ASSEMBLY_ASSEMBLY_SIM_MODELS_H__
