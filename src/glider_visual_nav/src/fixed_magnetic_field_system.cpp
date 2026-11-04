// Deterministic Gazebo sensor backend: honor <world><magnetic_field> in ENU/Tesla.
// The stock Harmonic system overwrites this value from its geographic lookup tables.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Magnetometer.hh>
#include <gz/sim/components/MagneticField.hh>
#include <gz/sim/components/World.hh>
#include <gz/sensors/MagnetometerSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <unordered_map>

namespace glider_visual_nav
{
class FixedMagneticFieldSystem : public gz::sim::System, public gz::sim::ISystemPostUpdate
{
public:
  void PostUpdate(const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {return;}
    const auto world = ecm.EntityByComponents(gz::sim::components::World());
    const auto field = ecm.Component<gz::sim::components::MagneticField>(world);
    if (!field) {return;}
    ecm.Each<gz::sim::components::Magnetometer>([&](const gz::sim::Entity & entity,
      const gz::sim::components::Magnetometer * component) {
        auto it = sensors_.find(entity);
        if (it == sensors_.end()) {
          auto description = component->Data();
          description.SetName(gz::sim::scopedName(entity, ecm, "::", false));
          if (description.Topic().empty()) {description.SetTopic(gz::sim::scopedName(entity, ecm) + "/magnetometer");}
          auto sensor = factory_.CreateSensor<gz::sensors::MagnetometerSensor>(description);
          if (!sensor) {return true;}
          it = sensors_.emplace(entity, std::move(sensor)).first;
        }
        it->second->SetWorldPose(gz::sim::worldPose(entity, ecm));
        it->second->SetWorldMagneticField(field->Data());
        it->second->Update(info.simTime, false);
        return true;
      });
  }
private:
  gz::sensors::SensorFactory factory_;
  std::unordered_map<gz::sim::Entity, std::unique_ptr<gz::sensors::MagnetometerSensor>> sensors_;
};
}  // namespace glider_visual_nav
GZ_ADD_PLUGIN(glider_visual_nav::FixedMagneticFieldSystem, gz::sim::System, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(glider_visual_nav::FixedMagneticFieldSystem, "glider_visual_nav::FixedMagneticFieldSystem")
