#ifndef __ASSEMBLY_SIM_ASSEMBLY_SOUP_PLUGIN_H
#define __ASSEMBLY_SIM_ASSEMBLY_SOUP_PLUGIN_H

#include <queue>

#include <ros/ros.h>

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

#include "models.h"

namespace assembly_sim {

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
      void queueStateUpdates();
      bool running_;

    protected:
      size_t mate_id_counter_;
      size_t atom_id_counter_;

      double max_trans_err_;
      double max_rot_err_;

      std::map<std::string, MateFactoryBasePtr> mate_factories_;
      std::map<std::string, MateModelPtr> mate_models_;
      std::map<std::string, AtomModelPtr> atom_models_;

      std::vector<AtomPtr> atoms_;

      // all mates
      boost::unordered_set<MatePtr> mates_;

      // mates to attach/detach in OnUpdate thread
      std::queue<MatePtr> mate_update_queue_;

      // for broadcasting coordinate transforms
      bool broadcast_tf_;
      std::string tf_world_frame_;

      clock_t last_tick_;
      int updates_per_second_;

      gazebo::common::Time last_update_time_;

  };
}

#endif // ifndef __ASSEMBLY_SIM_ASSEMBLY_SOUP_PLUGIN_H
