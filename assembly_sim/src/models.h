#ifndef __LCSR_ASSEMBLY_ASSEMBLY_SIM_MODELS_H__
#define __LCSR_ASSEMBLY_ASSEMBLY_SIM_MODELS_H__

#include "util.h"

namespace assembly_sim {

  struct MateModel;
  struct AtomModel;

  struct MateFactoryBase;

  struct Mate;
  struct MatePoint;
  struct Atom;

  typedef boost::shared_ptr<MateModel> MateModelPtr;
  typedef boost::shared_ptr<AtomModel> AtomModelPtr;

  typedef boost::shared_ptr<Mate> MatePtr;
  typedef boost::shared_ptr<MatePoint> MatePointPtr;
  typedef boost::shared_ptr<MateFactoryBase> MateFactoryBasePtr;
  typedef boost::shared_ptr<Atom> AtomPtr;

  // The model for a type of mate
  struct MateModel
  {
    MateModel(
        std::string type_,
        sdf::ElementPtr mate_elem_) :
      type(type_),
      mate_elem(mate_elem_)
    {
      // Get the mate template joint
      joint_template_sdf = boost::make_shared<sdf::SDF>();
      sdf::init(sdf::SDFPtr(joint_template_sdf));
      sdf::readString(complete_sdf(mate_elem->GetElement("joint")->ToString("")), joint_template_sdf);
      joint_template = joint_template_sdf->root->GetElement("model")->GetElement("joint");

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
                symmetries.push_back(KDL::Frame(Rx*Ry*Rz, KDL::Vector(0,0,0)));
              }
            }
          }
        }
      }

      // Add the identity if no symmetries were added
      if(symmetries.size() == 0) {
        symmetries.push_back(KDL::Frame::Identity());
      }
    }

    std::string type;

    // Transforms from the base mate frame to alternative frames
    std::vector<KDL::Frame> symmetries;

    // The mate sdf parameters
    sdf::ElementPtr mate_elem;

    // The sdf template for the joint to be created
    boost::shared_ptr<sdf::SDF> joint_template_sdf;
    sdf::ElementPtr joint_template;
  };

  struct MateFactoryBase
  {
    virtual MatePtr createMate(
        MatePointPtr female_mate_point,
        MatePointPtr male_mate_point,
        AtomPtr female_atom,
        AtomPtr male_atom) = 0;
  };

  template <class MateType>
    struct MateFactory : public MateFactoryBase
  {
    MateFactory(
        MateModelPtr mate_model_,
        gazebo::physics::ModelPtr gazebo_model_) :
      mate_model(mate_model_),
      gazebo_model(gazebo_model_)
    {}

    MateModelPtr mate_model;
    gazebo::physics::ModelPtr gazebo_model;

    virtual MatePtr createMate(
        MatePointPtr female_mate_point,
        MatePointPtr male_mate_point,
        AtomPtr female_atom,
        AtomPtr male_atom)
    {
      return boost::make_shared<MateType>(
          mate_model,
          gazebo_model,
          female_mate_point,
          male_mate_point,
          female_atom,
          male_atom);
    }
  };

  // This is a male or female point at which a given mate is located
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
      MateModelPtr mate_model,
      gazebo::physics::ModelPtr gazebo_model_,
      MatePointPtr female_mate_point_,
      MatePointPtr male_mate_point_,
      AtomPtr female_atom,
      AtomPtr male_atom);

    // Determine if the mate's attachment state needs to be updated
    // Asynchronous with Gazebo's update loop
    virtual void queueUpdate() = 0;

    // Update the constraint connectivity
    virtual void updateConstraints() = 0;

    // Update calculations needed to be done every tick
    virtual void update(gazebo::common::Time timestep) = 0;

    // Update functions
    void requestUpdate(State new_pending_state) { pending_state = new_pending_state; }
    bool needsUpdate() const { return pending_state != NONE; }
    void serviceUpdate() { pending_state = NONE; }
    State getUpdate() { return pending_state; }

    // Introspection
    std::string description;
    std::string getDescription() {
      return description;
    }

    // Mate model (same as mate points)
    MateModelPtr model;

    // Attachment states
    Mate::State state, pending_state;

    // Joint SDF
    sdf::ElementPtr joint_sdf;
    // Joint associated with mate
    // If this is NULL then the mate is unoccupied
    gazebo::physics::JointPtr joint;
    gazebo::physics::ModelPtr gazebo_model;

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

  // A mate
  struct ProximityMateBase : public Mate
  {
    // Threshold for attaching a mate
    double attach_threshold_linear;
    double attach_threshold_angular;

    // Threshold for detaching a mate
    double detach_threshold_linear;
    double detach_threshold_angular;

    std::vector<KDL::Frame>::iterator mated_symmetry;
    KDL::Twist mate_error;

    Eigen::Vector3d max_force, max_torque;

    ProximityMateBase(
        MateModelPtr mate_model,
        gazebo::physics::ModelPtr gazebo_model,
        MatePointPtr female_mate_point_,
        MatePointPtr male_mate_point_,
        AtomPtr female_atom,
        AtomPtr male_atom) :
      Mate(mate_model, gazebo_model, female_mate_point_, male_mate_point_, female_atom, male_atom),
      mated_symmetry(mate_model->symmetries.end()),
      max_force(Eigen::Vector3d::Zero()),
      max_torque(Eigen::Vector3d::Zero())
    {
      this->load_proximity_params();
    }

  protected:
    virtual void load_proximity_params()
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

      gazebo::math::Vector3 gz_max_force, gz_max_torque;
      sdf::ElementPtr max_force_elem = mate_elem->GetElement("max_force");
      if(max_force_elem) {
        max_force_elem->GetValue()->Get(gz_max_force);
        to_eigen(gz_max_force, max_force);
      }
      sdf::ElementPtr max_torque_elem = mate_elem->GetElement("max_torque");
      if(max_torque_elem) {
        max_torque_elem->GetValue()->Get(gz_max_torque);
        to_eigen(gz_max_torque, max_torque);
      }
    }

    virtual void attach()
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
      gzwarn<<">> mate error: "<<this->mate_error.vel.Norm()<<", "<<this->mate_error.rot.Norm()<<std::endl;
    }

    virtual void detach()
    {
      // Simply detach joint
      joint->Detach();
    }
  };

  struct ProximityMate : public ProximityMateBase
  {
    ProximityMate(
        MateModelPtr mate_model,
        gazebo::physics::ModelPtr gazebo_model,
        MatePointPtr female_mate_point_,
        MatePointPtr male_mate_point_,
        AtomPtr female_atom,
        AtomPtr male_atom) :
      ProximityMateBase(mate_model, gazebo_model, female_mate_point_, male_mate_point_, female_atom, male_atom)
    {
    }

    virtual void queueUpdate()
    {
      // Convenient references
      AtomPtr &female_atom = female;
      AtomPtr &male_atom = male;

      // Only analyze mates with associated joints
      if(!joint) {
        gzwarn<<"No joint for mate from "<<female_atom->link->GetName()<<" -> "<<male_atom->link->GetName()<<std::endl;
        return;
      }

      KDL::Frame female_atom_frame;
      to_kdl(female_atom->link->GetWorldPose(), female_atom_frame);

      KDL::Frame male_atom_frame;
      to_kdl(male_atom->link->GetWorldPose(), male_atom_frame);

      // Iterate over all symmetric mating positions
      for(std::vector<KDL::Frame>::iterator it_sym = model->symmetries.begin();
          it_sym != model->symmetries.end();
          ++it_sym)
      {
        // Compute the world frame of the female mate frame
        // This takes into account symmetries in the mate
        KDL::Frame female_mate_frame = female_atom_frame * female_mate_point->pose * (*it_sym);

        // Compute the world pose of the male mate frame
        // This takes into account the attachment displacement (anchor_offset)
        KDL::Frame male_mate_frame = male_atom_frame * male_mate_point->pose * anchor_offset;

        // compute twist between the two mate points
        KDL::Twist twist_err = diff(female_mate_frame, male_mate_frame);
        //gzwarn<<female_mate_point->pose.M<<std::endl;
        //gzwarn<<twist_err.vel.Norm()<<", "<<twist_err.rot.Norm()<<" VS "<<this->mate_error.vel.Norm()<<", "<<this->mate_error.rot.Norm()<<std::endl;

        if(state == Mate::MATED and it_sym == mated_symmetry)
        {
          gazebo::physics::JointWrench joint_wrench = joint->GetForceTorque(0);
          Eigen::Vector3d force, torque;
          to_eigen(joint_wrench.body1Force, force);
          to_eigen(joint_wrench.body1Torque, torque);

          //gzwarn<<">>> "<<this->getDescription()<<" Force: "<<force<<" Torque: "<<torque<<std::endl;
          // Determine if active mate needs to be detached
          if(twist_err.vel.Norm() > detach_threshold_linear or
             twist_err.rot.Norm() > detach_threshold_angular or
             ((max_force.array() > 0.0).any() and (force.array().abs() > max_force.array()).any()) or
             ((max_torque.array() > 0.0).any() and (torque.array().abs() > max_torque.array()).any()))
          {
            // The mate points are beyond the detach threhold and should be demated
            gzwarn<<"> Request unmate "<<getDescription()<<std::endl;
            this->requestUpdate(Mate::UNMATED);
            break;
          } else if(
              twist_err.vel.Norm() / mate_error.vel.Norm() < 0.8 and
              twist_err.rot.Norm() / mate_error.rot.Norm() < 0.8)
          {
            // Re-mate but closer to the desired point
            gzwarn<<"> Request remate "<<getDescription()<<std::endl;
            this->mate_error = twist_err;
            this->requestUpdate(Mate::MATED);
            break;
          }
        } else {
          // Determine if mated atoms need to be attached
          if(twist_err.vel.Norm() < attach_threshold_linear and
             twist_err.rot.Norm() < attach_threshold_angular)
          {
            // The mate points are within the attach threshold and should be mated
            gzwarn<<"> Request mate "<<getDescription()<<std::endl;
            this->mated_symmetry = it_sym;
            this->mate_error = twist_err;
            this->requestUpdate(Mate::MATED);
            break;
          }
        }
      }
    }

    virtual void updateConstraints()
    {
      if(not this->needsUpdate()) {
        this->queueUpdate();
      }

      if(not this->needsUpdate()) {
        return;
      }

      switch(this->getUpdate()) {
        case Mate::NONE:
          break;

        case Mate::UNMATED:
          if(state == Mate::MATED) {
            gzwarn<<"> Detaching "<<female->link->GetName()<<" from "<<male->link->GetName()<<"!"<<std::endl;
            this->detach();
            this->state = Mate::UNMATED;
            this->mated_symmetry = model->symmetries.end();
          }
          break;

        case Mate::MATING:
          gzwarn<<"ProximityMate does not support Mate::MATING"<<std::endl;
          break;

        case Mate::MATED:
          if(state != Mate::MATED) {
            gzwarn<<"> Attaching "<<female->link->GetName()<<" to "<<male->link->GetName()<<"!"<<std::endl;
          } else {
            gzwarn<<"> Reattaching "<<female->link->GetName()<<" to "<<male->link->GetName()<<"!"<<std::endl;
          }
          this->attach();
          this->state = Mate::MATED;
          break;
      };

      this->serviceUpdate();
    }

    virtual void update(gazebo::common::Time timestep)
    {
    }

  };

  struct DipoleMate : public ProximityMate
  {
    //TODO: Remove these, unused
    double min_force_linear, min_force_angular, min_force_linear_deadband, min_force_angular_deadband;
    double max_force_linear, max_force_angular, max_force_linear_deadband, max_force_angular_deadband;
    double moment;

    // Individual magnetic dipole parameters
    struct Dipole {
      double min_distance;
      KDL::Vector position;
      KDL::Vector moment;
    };

    // Dipoles involved in this mate
    std::vector<Dipole> dipoles;

    DipoleMate(
        MateModelPtr mate_model,
        gazebo::physics::ModelPtr gazebo_model,
        MatePointPtr female_mate_point_,
        MatePointPtr male_mate_point_,
        AtomPtr female_atom,
        AtomPtr male_atom) :
      ProximityMate(mate_model, gazebo_model, female_mate_point_, male_mate_point_, female_atom, male_atom)
    {
      this->load();
    }

    virtual void load()
    {
      sdf::ElementPtr mate_elem = model->mate_elem;

      // Get force attach/detach thresholds
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
      sdf::ElementPtr dipole_elem = mate_elem->GetElement("dipole");

      while(dipole_elem && dipole_elem->GetName() == "dipole")
      {
        gazebo::math::Vector3 position_gz, moment_gz;
        double min_distance;

        // Get the position of the dipole
        sdf::ElementPtr position_elem = dipole_elem->GetElement("position");
        position_elem->GetValue()->Get(position_gz);

        // Get the magnetic moment
        sdf::ElementPtr moment_elem = dipole_elem->GetElement("moment");
        moment_elem->GetValue()->Get(moment_gz);

        // Get minimum distance
        sdf::ElementPtr min_dist_elem = dipole_elem->GetElement("min_distance");
        min_dist_elem->GetValue()->Get(min_distance);

        Dipole dipole;
        dipole.position = KDL::Vector(position_gz.x, position_gz.y, position_gz.z);
        dipole.moment = KDL::Vector(moment_gz.x, moment_gz.y, moment_gz.z);
        dipole.min_distance = min_distance;

        dipoles.push_back(dipole);

        // Get the next dipole element
        dipole_elem = dipole_elem->GetNextElement(dipole_elem->GetName());
      }
    }

    virtual void update(gazebo::common::Time timestep)
    {
      // Convenient references
      AtomPtr &female_atom = this->female;
      AtomPtr &male_atom = this->male;

      MatePointPtr &female_mate_point = this->female_mate_point;
      MatePointPtr &male_mate_point = this->male_mate_point;

      // Don't apply magnetic force if the mate is attached
      if(state == Mate::MATED) {
        return;
      }

      // Compute the world pose of the female mate frame
      KDL::Frame female_atom_frame;
      to_kdl(female_atom->link->GetWorldPose(), female_atom_frame);
      KDL::Frame female_mate_frame = female_atom_frame * female_mate_point->pose;
      gazebo::math::Pose female_mate_pose;
      to_gazebo(female_mate_frame, female_mate_pose);

      // Compute the world pose of the male mate frame
      // This takes into account the attachment displacement (anchor_offset)
      KDL::Frame male_atom_frame;
      to_kdl(male_atom->link->GetWorldPose(), male_atom_frame);
      KDL::Frame male_mate_frame = male_atom_frame * male_mate_point->pose * this->anchor_offset;
      gazebo::math::Pose male_mate_pose;
      to_gazebo(male_mate_frame, male_mate_pose);

      // compute twist between the two mate points to determine if we need to simulate dipole interactions
      mate_error = diff(female_mate_frame, male_mate_frame);
      if(mate_error.vel.Norm() > 0.03) {
        return;
      } else {
        //gzwarn<<"mate "<<description<<" attracting"<<std::endl;
      }

      // compute and apply forces between all male/female pairs of dipoles
      for(std::vector<Dipole>::iterator it_fdp=dipoles.begin(); it_fdp!=dipoles.end(); ++it_fdp)
      {
        for(std::vector<Dipole>::iterator it_mdp=dipoles.begin(); it_mdp!=dipoles.end(); ++it_mdp)
        {
          KDL::Frame female_dipole_frame(female_mate_frame.M, female_mate_frame*it_fdp->position);
          KDL::Frame male_dipole_frame(male_mate_frame.M, male_mate_frame*it_mdp->position);

          gazebo::math::Pose female_dipole_pose;
          to_gazebo(female_dipole_frame, female_dipole_pose);
          gazebo::math::Pose male_dipole_pose;
          to_gazebo(male_dipole_frame, male_dipole_pose);

          // compute twist between the two mate points
          KDL::Twist twist_err = diff(female_dipole_frame, male_dipole_frame);

          // Compute dipole force
          static const double mu0 = 4*M_PI*1E-7;

          KDL::Vector r = twist_err.vel;
          double rn = r.Norm();
          rn = std::max(it_fdp->min_distance, rn);
          r = rn / r.Norm() * r;

          KDL::Vector rh = (rn > 1E-5) ? twist_err.vel/rn : KDL::Vector(1,0,0);

          // Compute magnetic moments and fields
          KDL::Vector
            m1 = female_dipole_frame.M * it_fdp->moment,
            m2 = male_dipole_frame.M * it_mdp->moment,
            B1 = mu0 / 4 / M_PI / pow(rn,3) * ( 3 * (KDL::dot(m1,rh)*rh - m1)),
            B2 = mu0 / 4 / M_PI / pow(rn,3) * ( 3 * (KDL::dot(m2,rh)*rh - m2));

          // Compute wrenches in the world frame applied at the dipole point
          KDL::Wrench
            W1(-3 * mu0 / 4 / M_PI / pow(rn,4) * ( (rh*m2)*m1 + (rh*m1)*m2 - 2*rh*KDL::dot(m1,m2) + 5*rh*KDL::dot(rh*m2,rh*m1) ), m1 * B2),
            W2(-W1.force, m2 * B1);

          // Convert to wrenches applied at centers of mass
          KDL::Wrench
            W1cog(W1),
            W2cog(W2);

          W1.force -= W1.torque * it_fdp->position / pow(it_fdp->position.Norm(),2.0);
          W2.force -= W2.torque * it_mdp->position / pow(it_mdp->position.Norm(),2.0);

          //gzwarn<<"Dipole force: "<<std::setprecision(4)<<std::fixed
            //<<"female("<<female_atom->link->GetName()<<"#"<<female_mate_point->id<<") "<<[>female_mate_frame<<<]" --> "<<m1<<" F1="<<W1.force
            //<<" - "
            //<<"  male("<<male_atom->link->GetName()<<"#"<<male_mate_point->id<<") "<<[>male_mate_frame<<<]" --> "<<m2<<" F2="<<W2.force
            //<<""<<std::endl;

          // Apply force to links
          gazebo::math::Vector3 F1gz, F2gz, T1gz, T2gz;

          to_gazebo(W1cog, F1gz, T1gz);
          to_gazebo(W2cog, F2gz, T2gz);

          female_atom->link->AddForceAtWorldPosition(F1gz, female_dipole_pose.pos);
          female_atom->link->AddTorque(T1gz);

          male_atom->link->AddForceAtWorldPosition(F2gz, male_dipole_pose.pos);
          male_atom->link->AddTorque(T2gz);
        }
      }
    }
  };
}

#endif // ifndef __LCSR_ASSEMBLY_ASSEMBLY_SIM_MOgazebo::common::Time timestepDELS_H__
