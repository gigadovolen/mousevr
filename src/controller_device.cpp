#include "controller_device.h"
#include <string>

ControllerDevice::ControllerDevice(vr::ETrackedControllerRole role) : role_(role), device_id_(vr::k_unTrackedDeviceIndexInvalid) {
    
    // Initialize pose with safe defaults
    memset(&pose_, 0, sizeof(pose_));
    pose_.poseIsValid = true;
    pose_.result = vr::TrackingResult_Running_OK;
    pose_.deviceIsConnected = true;

    // Set required quaternion components
    pose_.qWorldFromDriverRotation.w = 1.0f;
    pose_.qDriverFromHeadRotation.w = 1.0f;
    pose_.qRotation.w = 1.0f;

};

vr::EVRInitError ControllerDevice::Activate(uint32_t unObjectId) {
    vr::VRDriverLog()->Log("ControllerDevice::Activate");

    const vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);
    vr::VRProperties()->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, role_);

    vr::VRProperties()->SetStringProperty(container, vr::Prop_InputProfilePath_String,
        "{mousevr}/resources/input/keyboard_profile.json");

    vr::VRDriverInput()->CreateScalarComponent(container, "/input/joystick/x", &keyboard_inputs[kInputHandle_joystick_x],
        vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    vr::VRDriverInput()->CreateScalarComponent(container, "/input/joystick/y", &keyboard_inputs[kInputHandle_joystick_y],
        vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);

    vr::VRDriverInput()->CreateScalarComponent(container, "/input/trigger/value", &keyboard_inputs[kInputHandle_trigger_value],
        vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/trigger/click", &keyboard_inputs[kInputHandle_trigger_click]);

    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/grip/click", &keyboard_inputs[kInputHandle_grip_click]);

    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/z/click", &keyboard_inputs[kInputHandle_z_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/v/click", &keyboard_inputs[kInputHandle_v_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/y/click", &keyboard_inputs[kInputHandle_y_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/r/click", &keyboard_inputs[kInputHandle_r_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/c/click", &keyboard_inputs[kInputHandle_c_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/f/click", &keyboard_inputs[kInputHandle_f_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/q/click", &keyboard_inputs[kInputHandle_q_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/e/click", &keyboard_inputs[kInputHandle_e_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/o/click", &keyboard_inputs[kInputHandle_o_click]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/p/click", &keyboard_inputs[kInputHandle_p_click]);

    device_id_ = unObjectId;

    return vr::VRInitError_None;
}

void ControllerDevice::Deactivate() {
    device_id_ = vr::k_unTrackedDeviceIndexInvalid;
}

void ControllerDevice::EnterStandby() {
}

void* ControllerDevice::GetComponent(const char* pchComponentNameAndVersion) {
    return nullptr;
}

void ControllerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) {
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t ControllerDevice::GetPose() {
    std::lock_guard<std::mutex> lock(pose_mutex_);

    // Get HMD pose as reference (following tutorial pattern)
    vr::TrackedDevicePose_t hmd_pose{};
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, &hmd_pose, 1);

    if (hmd_pose.bPoseIsValid)
    {
        // Position controller relative to HMD
        pose_.vecPosition[0] = hmd_pose.mDeviceToAbsoluteTracking.m[0][3];
        pose_.vecPosition[1] = hmd_pose.mDeviceToAbsoluteTracking.m[1][3];
        pose_.vecPosition[2] = hmd_pose.mDeviceToAbsoluteTracking.m[2][3];

        // Copy HMD orientation (optional - you can modify this based on input)
        const vr::HmdQuaternion_t hmd_orientation = HmdQuaternion_FromMatrix(hmd_pose.mDeviceToAbsoluteTracking);
        pose_.qRotation = hmd_orientation;
    }
    else
    {
        // Fallback to fixed position if HMD pose is invalid
        pose_.vecPosition[0] = 0;
        pose_.vecPosition[1] = 1.2f; // Chest level
        pose_.vecPosition[2] = -0.3f; // In front
    }

    return pose_;
}

void ControllerDevice::RunFrame(const Inputs& inputs)
{
    if (device_id_ != vr::k_unTrackedDeviceIndexInvalid)
    {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_id_, GetPose(), sizeof(vr::DriverPose_t));

        vr::IVRDriverInput* driver_input = vr::VRDriverInput();
        if (driver_input) {
            float x = 0.0f, y = 0.0f;

            if (inputs.aPressed) x -= 1.0f;
            if (inputs.dPressed) x += 1.0f;
            if (inputs.wPressed) y += 1.0f;
            if (inputs.sPressed) y -= 1.0f;

            // NORMALIZATION
            if (x != 0.0f && y != 0.0f)
            {
                float len = sqrtf(x * x + y * y);
                x /= len;
                y /= len;
            }

            driver_input->UpdateScalarComponent(keyboard_inputs[kInputHandle_joystick_x], x, 0);
            driver_input->UpdateScalarComponent(keyboard_inputs[kInputHandle_joystick_y], y, 0);
            
            driver_input->UpdateScalarComponent(keyboard_inputs[kInputHandle_trigger_value], inputs.SPACEPressed ? 1.0f : 0.0f, 0);
            driver_input->UpdateBooleanComponent(keyboard_inputs[kInputHandle_trigger_click], inputs.SPACEPressed, 0);
            driver_input->UpdateBooleanComponent(keyboard_inputs[kInputHandle_grip_click], inputs.ESCPressed, 0);

            driver_input->UpdateBooleanComponent(keyboard_inputs[kInputHandle_z_click], inputs.zPressed, 0);
            driver_input->UpdateBooleanComponent(keyboard_inputs[kInputHandle_v_click], inputs.vPressed, 0);
            driver_input->UpdateBooleanComponent(keyboard_inputs[kInputHandle_y_click], inputs.yPressed, 0);
            driver_input->UpdateBooleanComponent(keyboard_inputs[kInputHandle_r_click], inputs.rPressed, 0);
            driver_input->UpdateBooleanComponent(keyboard_inputs[kInputHandle_c_click], inputs.cPressed, 0);
            driver_input->UpdateBooleanComponent(keyboard_inputs[kInputHandle_f_click], inputs.fPressed, 0);
            driver_input->UpdateBooleanComponent(keyboard_inputs[kInputHandle_q_click], inputs.qPressed, 0);
            driver_input->UpdateBooleanComponent(keyboard_inputs[kInputHandle_e_click], inputs.ePressed, 0);
            driver_input->UpdateBooleanComponent(keyboard_inputs[kInputHandle_o_click], inputs.oPressed, 0);
            driver_input->UpdateBooleanComponent(keyboard_inputs[kInputHandle_p_click], inputs.pPressed, 0);
        }
        
    }
}

// Utility function from OpenVR samples (for matrix to quaternion conversion)
vr::HmdQuaternion_t ControllerDevice::HmdQuaternion_FromMatrix(const vr::HmdMatrix34_t& matrix)
{
    vr::HmdQuaternion_t q{};

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;

    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

    return q;
}