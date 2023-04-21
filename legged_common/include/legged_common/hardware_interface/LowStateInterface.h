#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace legged{
class LowStateHandle{
  public:
    LowStateHandle() = default;

    std::vector get()
    {
      std::vector
    }

  private:
    double* MotorPos = {nullptr};
    double* MotorVel = {nullptr};
    double* MotorToq = {nullptr};

    const bool* isContact_ = {nullptr};

    double* orientation = {nullptr};
    double* angularVel = {nullptr};
    double* linearAcc = {nullptr};

}


}


