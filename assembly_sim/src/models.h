#ifndef __LCSR_ASSEMBLY_ASSEMBLY_SIM_MODELS_H__
#define __LCSR_ASSEMBLY_ASSEMBLY_SIM_MODELS_H__

#include "assembly_soup_plugin.h"
#include "util.h"

namespace assembly_sim {

  struct ProximityMateModel : public MateModel
  {
    ProximityMateModel(std::string type) : MateModel(type) {}

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

    virtual MateModel::State getStateUpdate(const MatePtr mate)
    {
      MateModel::State new_state = MateModel::NONE;

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
        //if(female_mate_point->model->id == 0) gzwarn<<" --- err linear: "<<twist_err.vel.Norm()<<" angular: "<<twist_err.rot.Norm()<<std::endl;

        // Determine if mated atoms need to be detached
        if(mate->joint->GetParent() and mate->joint->GetChild()) {
          //gzwarn<<"Parts are mated!"<<std::endl;

          if(twist_err.vel.Norm() > detach_threshold_linear or
             twist_err.rot.Norm() > detach_threshold_angular)
          {
            new_state = MateModel::UNMATED;
          } else if(
              twist_err.vel.Norm() < mate->mate_error.vel.Norm() and
              twist_err.rot.Norm() < mate->mate_error.rot.Norm())
          {
            // re-mate but closer to the desired point
            mate->mate_error = twist_err;
            new_state = MateModel::MATED;
          }

        } else {
          //gzwarn<<"Parts are not mated!"<<std::endl;
          if(twist_err.vel.Norm() < attach_threshold_linear and
             twist_err.rot.Norm() < attach_threshold_angular)
          {
            mate->mate_error = twist_err;
            new_state = MateModel::MATED;
          }
        }

      } else {
        gzwarn<<"No joint for mate from "<<female_atom->link->GetName()<<" -> "<<male_atom->link->GetName()<<std::endl;
      }

      return new_state;
    }

    virtual void updateState(MatePtr mate)
    {
      switch(mate->pending_state) {
        case MateModel::NONE:
          break;

        case MateModel::UNMATED:
          if(mate->state == MateModel::MATED) {
            gzwarn<<"Detaching "<<mate->female->link->GetName()<<" from "<<mate->male->link->GetName()<<"!"<<std::endl;
            this->detach(mate);
          }
          break;

        case MateModel::MATING:
          gzwarn<<"ProximityMate does not support MateModel::MATING"<<std::endl;
          break;

        case MateModel::MATED:
          if(mate->state != MateModel::MATED) {
            gzwarn<<"Attaching "<<mate->female->link->GetName()<<" to "<<mate->male->link->GetName()<<"!"<<std::endl;
          } else {
            gzwarn<<"Reattaching "<<mate->female->link->GetName()<<" to "<<mate->male->link->GetName()<<"!"<<std::endl;
          }
          this->attach(mate);
          break;
      };

      mate->pending_state = MateModel::NONE;
    }

    void attach(MatePtr mate)
    {
#if 0
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
#else
      // Get the male atom frame
      KDL::Frame male_atom_frame;
      to_kdl(mate->male->link->GetWorldPose(), male_atom_frame);

      // detach the atoms if they're already attached (they're going to be re-attached)
      this->detach(mate);

      // attach two atoms via joint
      mate->joint->Attach(mate->female->link, mate->male->link);

      // set stiffness based on proximity to goal
      double lin_err = mate->mate_error.vel.Norm();
      double ang_err = mate->mate_error.rot.Norm();
      double max_lin_err = detach_threshold_linear;
      double max_ang_err = detach_threshold_angular;
      //mate->joint->SetAttribute(
      //"stop_erp", 0,
      //mate->max_stop_erp *
      //std::min(std::max(0.0, max_lin_err - lin_err), max_lin_err) / max_lin_err *
      //std::min(std::max(0.0, max_ang_err - ang_err), max_ang_err) / max_ang_err);
      //mate->joint->SetAttribute(
      //"erp", 0,
      //mate->max_erp *
      //std::min(std::max(0.0, max_lin_err - lin_err), max_lin_err) / max_lin_err *
      //std::min(std::max(0.0, max_ang_err - ang_err), max_ang_err) / max_ang_err);

      //mate->joint->SetHighStop(0, lin_err);

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

      //gzwarn<<" ---- initial anchor pose: "<<std::endl<<initial_anchor_frame<<std::endl;
      //gzwarn<<" ---- actual anchor pose: "<<std::endl<<actual_anchor_frame<<std::endl;
      gzwarn<<" ---- mate error: "<<mate->mate_error.vel.Norm()<<", "<<mate->mate_error.rot.Norm()<<std::endl;

      mate->state = MateModel::MATED;
#endif
    }

    void detach(MatePtr mate)
    {
      // Simply detach joint
      mate->joint->Detach();

      mate->state = MateModel::UNMATED;
    }
  };

  struct DipoleMateModel : public ProximityMateModel
  {
    DipoleMateModel(std::string type) : ProximityMateModel(type) {}
  };

#if 0
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

    virtual MateModel::State getStateUpdate(const MatePtr mate)
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

          if(twist_err.vel.Norm() > detach_threshold_linear or
             twist_err.rot.Norm() > detach_threshold_angular)
          {
            mates_to_detach.push_back(mate);
          }

        } else {
          //gzwarn<<"Parts are not mated!"<<std::endl;
          if(twist_err.vel.Norm() < attach_threshold_linear and
             twist_err.rot.Norm() < attach_threshold_angular)
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
#endif

}

#endif // ifndef __LCSR_ASSEMBLY_ASSEMBLY_SIM_MODELS_H__
