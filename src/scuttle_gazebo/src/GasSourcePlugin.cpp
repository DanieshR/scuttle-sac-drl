#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/float.pb.h>

using namespace gz;
using namespace sim;
using namespace systems;

class GasSourcePlugin
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventMgr*/) override
  {
    if (_sdf->HasElement("gas_source_x")) this->source_x = _sdf->Get<double>("gas_source_x");
    if (_sdf->HasElement("gas_source_y")) this->source_y = _sdf->Get<double>("gas_source_y");
    
    this->pub = this->node.Advertise<gz::msgs::Float>("/gas_sensor");
    gzmsg << "[GasSourcePlugin] Initialized. Source at: " << this->source_x << ", " << this->source_y << std::endl;
  }

  public: void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm) override
  {
    if (!this->robot_found) {
      _ecm.Each<components::Name, components::Pose>(
        [&](const Entity & _entity, const components::Name * _name, const components::Pose * _pose)->bool {
          if (_name->Data() == "scuttle") {
            this->robot_entity = _entity;
            this->robot_found = true;
            return false;
          }
          return true;
        });
        return;
    }

    auto pose_comp = _ecm.Component<components::Pose>(this->robot_entity);
    if (!pose_comp) return;

    math::Pose3d robot_pose = pose_comp->Data();
    double dist_sq = std::pow(robot_pose.Pos().X() - this->source_x, 2) + 
                     std::pow(robot_pose.Pos().Y() - this->source_y, 2);
    
    // Gaussian distribution gas spread
    double intensity = 100.0 * std::exp(-dist_sq / 5.0);

    gz::msgs::Float msg;
    msg.set_data(intensity);
    this->pub.Publish(msg);
  }

  private:
    double source_x = 0.0, source_y = 0.0;
    bool robot_found = false;
    Entity robot_entity;
    gz::transport::Node node;
    gz::transport::Node::Publisher pub;
};

GZ_ADD_PLUGIN(GasSourcePlugin, gz::sim::System, GasSourcePlugin::ISystemConfigure, GasSourcePlugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(GasSourcePlugin, "gas_mapping::GasSourcePlugin")