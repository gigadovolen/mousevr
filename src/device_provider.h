#pragma once

#include "openvr_driver.h"
#include "controller_device.h"
#include "hmd_device_driver.h"
#include <memory>

class DeviceProvider : public vr::IServerTrackedDeviceProvider {
public:
    vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override;
    void Cleanup() override;
    const char* const* GetInterfaceVersions() override;
    void RunFrame() override;
    bool ShouldBlockStandbyMode() override;
    void EnterStandby() override;
    void LeaveStandby() override;

private:
    std::unique_ptr<ControllerDevice> my_left_device_;
    std::unique_ptr<MyHMDControllerDeviceDriver> my_hmd_device_;

    long prev_mouse_x_ = 0;
    long prev_mouse_y_ = 0;
    bool first_mouse_update_ = true;
};