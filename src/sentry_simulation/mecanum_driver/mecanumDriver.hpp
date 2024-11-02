#ifndef IGNITION_GAZEBO_SYSTEMS_MECANUM_DRIVE_HH
#define IGNITION_GAZEBO_SYSTEMS_MECANUM_DRIVE_HH

#include <memory>
#include <ignition/gazebo/System.hh>

namespace ignition
{
    namespace gazebo
    {
        namespace systems
        {
            class MecanumDrive2Private;
            class IGNITION_GAZEBO_VISIBLE MecanumDrive2
                : public ignition::gazebo::System,
                  public ISystemConfigure,
                  public ISystemPreUpdate,
                  public ISystemPostUpdate
            {
            public:
                MecanumDrive2();
                ~MecanumDrive2() override = default;

            public:
                void Configure(const Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               EntityComponentManager &_ecm,
                               EventManager &_eventMgr) override;
                void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                               ignition::gazebo::EntityComponentManager &_ecm) override;
                void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                const ignition::gazebo::EntityComponentManager &_ecm) override;

            private:
                std::unique_ptr<MecanumDrive2Private> dataPtr;
            };
        } // namespace systems
    }     // namespace gazebo
} // namespace ignition

#endif //IGNITION_GAZEBO_SYSTEMS_MECANUM_DRIVE_HH