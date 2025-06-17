#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include "openvr_driver.h"
#include <mutex>
#include <array>

enum InputHandles {
    kInputHandle_joystick_x,
    kInputHandle_joystick_y,
    kInputHandle_trigger_click,
    kInputHandle_trigger_value,
    kInputHandle_grip_click,
    kInputHandle_z_click,
    kInputHandle_v_click,
    kInputHandle_y_click,
    kInputHandle_r_click,
    kInputHandle_c_click,
    kInputHandle_f_click,
    kInputHandle_q_click,
    kInputHandle_e_click,
    kInputHandle_o_click,
    kInputHandle_p_click,
    kInputHandle_COUNT
};

struct Inputs {
    bool ESCPressed;
    bool SPACEPressed;
    bool wPressed;
    bool aPressed;
    bool sPressed;
    bool dPressed;
    bool zPressed;
    bool vPressed;
    bool yPressed;
    bool rPressed;
    bool cPressed;
    bool fPressed;
    bool qPressed;
    bool ePressed;
    bool oPressed;
    bool pPressed;
};

class ControllerDevice : public vr::ITrackedDeviceServerDriver {
public:
    ControllerDevice(vr::ETrackedControllerRole role);

    // Inherited via ITrackedDeviceServerDriver
    virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
    virtual void Deactivate() override;
    virtual void EnterStandby() override;
    virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
    virtual vr::DriverPose_t GetPose() override;

    void RunFrame(const Inputs& inputs);

private:
    std::array<vr::VRInputComponentHandle_t, kInputHandle_COUNT> keyboard_inputs;

    vr::ETrackedControllerRole role_;
    vr::TrackedDeviceIndex_t device_id_;
    vr::PropertyContainerHandle_t property_container_;

    // Pose data
    vr::DriverPose_t pose_;
    std::mutex pose_mutex_;

    // Utility functions
    vr::HmdQuaternion_t HmdQuaternion_FromMatrix(const vr::HmdMatrix34_t& matrix);

};