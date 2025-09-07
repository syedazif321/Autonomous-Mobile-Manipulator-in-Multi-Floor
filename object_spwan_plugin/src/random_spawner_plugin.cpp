#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <random>
#include <string>
#include <vector>

namespace gazebo
{
  class RandomSpawnerPlugin : public WorldPlugin
  {
  public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      world_ = _world;
      node_ = gazebo_ros::Node::Get(_sdf);

      // Get model list from <model> tags in SDF
      if (_sdf->HasElement("model"))
      {
        sdf::ElementPtr elem = _sdf->GetElement("model");
        while (elem)
        {
          models_.push_back(elem->Get<std::string>());
          elem = elem->GetNextElement("model");
        }
      }

      if (models_.empty())
      {
        RCLCPP_WARN(node_->get_logger(), "No models provided, using default box/sphere");
        models_ = {"model://box", "model://sphere"};
      }

      // Random generators
      rng_.seed(std::random_device{}());
      dist_x_ = std::uniform_real_distribution<double>(-3.0, 3.0);
      dist_y_ = std::uniform_real_distribution<double>(-3.0, 3.0);
      dist_t_ = std::uniform_int_distribution<int>(3, 8);   // seconds between spawns
      dist_n_ = std::uniform_int_distribution<int>(1, 3);   // how many models per spawn

      update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RandomSpawnerPlugin::OnUpdate, this));

      last_spawn_time_ = world_->SimTime();
    }

  private:
    void OnUpdate()
    {
      auto sim_time = world_->SimTime();

      if ((sim_time - last_spawn_time_).Double() > next_spawn_interval_)
      {
        int num_to_spawn = dist_n_(rng_);
        for (int i = 0; i < num_to_spawn; i++)
          SpawnRandomModel();

        last_spawn_time_ = sim_time;
        next_spawn_interval_ = dist_t_(rng_);
      }

      // Remove expired models
      for (auto it = spawned_.begin(); it != spawned_.end(); )
      {
        if ((sim_time - it->spawn_time).Double() > it->lifetime)
        {
          world_->RemoveModel(it->name);
          it = spawned_.erase(it);
        }
        else
          ++it;
      }
    }

    void SpawnRandomModel()
    {
      // Pick random model
      std::string model = models_[rng_() % models_.size()];
      ignition::math::Pose3d pose(dist_x_(rng_), dist_y_(rng_), 0.1, 0, 0, 0);

      // Unique name
      std::string name = "rand_model_" + std::to_string(counter_++);

      sdf::SDF modelSDF;
      modelSDF.SetFromString(
        "<sdf version='1.6'>\
          <model name='" + name + "'>\
            <include>\
              <uri>" + model + "</uri>\
            </include>\
            <pose>" + std::to_string(pose.Pos().X()) + " " +
                        std::to_string(pose.Pos().Y()) + " " +
                        std::to_string(pose.Pos().Z()) + " 0 0 0</pose>\
          </model>\
        </sdf>");

      world_->InsertModelSDF(modelSDF);

      // Store with lifetime
      Spawned s;
      s.name = name;
      s.spawn_time = world_->SimTime();
      s.lifetime = 5 + (rng_() % 5); // 5–10 sec lifetime
      spawned_.push_back(s);
    }

    struct Spawned
    {
      std::string name;
      common::Time spawn_time;
      double lifetime;
    };

    physics::WorldPtr world_;
    gazebo_ros::Node::SharedPtr node_;
    event::ConnectionPtr update_connection_;

    std::vector<std::string> models_;
    std::vector<Spawned> spawned_;

    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_x_, dist_y_;
    std::uniform_int_distribution<int> dist_t_, dist_n_;

    common::Time last_spawn_time_;
    int next_spawn_interval_ = 5;
    int counter_ = 0;
  };

  GZ_REGISTER_WORLD_PLUGIN(RandomSpawnerPlugin)
}
