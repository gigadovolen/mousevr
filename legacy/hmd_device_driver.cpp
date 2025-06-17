#include "hmd_device_driver.h"
#include <string.h>
#include <cmath>
#include <thread>

#define M_PI 3.14159265358979323846

static const char* my_hmd_main_settings_section = "driver_simplehmd";
static const char* my_hmd_display_settings_section = "simplehmd_display";

MonitorHMDDisplayComponent::MonitorHMDDisplayComponent(const MonitorHMD& config) : config_(config) {}

void MonitorHMDDisplayComponent::GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) {
    *pnX = config_.window_x;
    *pnY = config_.window_y;
    *pnWidth = config_.window_width;
    *pnHeight = config_.window_height;
}

bool MonitorHMDDisplayComponent::IsDisplayOnDesktop() {
    return true;
}

bool MonitorHMDDisplayComponent::IsDisplayRealDisplay() {
    return false;
}

void MonitorHMDDisplayComponent::GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) {
    *pnWidth = config_.render_width;
    *pnHeight = config_.render_height;
}

void MonitorHMDDisplayComponent::GetEyeOutputViewport(vr::EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) {
    *pnY = 0;
    *pnWidth = config_.window_width / 2;
    *pnHeight = config_.window_height;
    if (eEye == vr::Eye_Left) {
        *pnX = 0;
    }
    else {
        *pnX = config_.window_width / 2;
    }
}

void MonitorHMDDisplayComponent::GetProjectionRaw(vr::EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom)
{
    *pfLeft = config_.field_of_view_left;
    *pfRight = config_.field_of_view_right;
    *pfTop = config_.field_of_view_top;
    *pfBottom = config_.field_of_view_bottom;
}

vr::DistortionCoordinates_t MonitorHMDDisplayComponent::ComputeDistortion(vr::EVREye eEye, float fU, float fV)
{
    vr::DistortionCoordinates_t coordinates{};
    coordinates.rfBlue[0] = fU;
    coordinates.rfBlue[1] = fV;
    coordinates.rfGreen[0] = fU;
    coordinates.rfGreen[1] = fV;
    coordinates.rfRed[0] = fU;
    coordinates.rfRed[1] = fV;
    return coordinates;
}

bool MonitorHMDDisplayComponent::ComputeInverseDistortion(
    vr::HmdVector2_t* pvecOutUndistorted,
    vr::EVREye eEye,
    uint32_t unDistortionFunc,
    float fU,
    float fV) {

    if (pvecOutUndistorted) {
        pvecOutUndistorted->v[0] = fU; // Undistorted U = Input U
        pvecOutUndistorted->v[1] = fV; // Undistorted V = Input V
    }
    return true;
}

MonitorHMDDevice::MonitorHMDDevice()
{
    is_active_ = false;

    char model_number[1024];
    vr::VRSettings()->GetString(my_hmd_main_settings_section, "model_number", model_number, sizeof(model_number));
    model_number_ = model_number;

    serial_number_ = "77777777";

    MonitorHMD display_config{};
    display_config.window_x = vr::VRSettings()->GetInt32(my_hmd_display_settings_section, "window_x");
    display_config.window_y = vr::VRSettings()->GetInt32(my_hmd_display_settings_section, "window_y");

    display_config.window_width = vr::VRSettings()->GetInt32(my_hmd_display_settings_section, "window_width");
    display_config.window_height = vr::VRSettings()->GetInt32(my_hmd_display_settings_section, "window_height");

    display_config.render_width = vr::VRSettings()->GetInt32(my_hmd_display_settings_section, "render_width");
    display_config.render_height = vr::VRSettings()->GetInt32(my_hmd_display_settings_section, "render_height");


    display_component_ = std::make_unique<MonitorHMDDisplayComponent>(display_config);
}

vr::EVRInitError MonitorHMDDevice::Activate(uint32_t unObjectId) {
    // Let's keep track of our device index. It'll be useful later.
    // Also, if we re-activate, be sure to set this.
    device_index_ = unObjectId;

    // Set a member to keep track of whether we've activated yet or not
    is_active_ = true;

    // For keeping track of frame number for animating motion.
    frame_number_ = 0;

    // Properties are stored in containers, usually one container per device index. We need to get this container to set
    // The properties we want, so we call this to retrieve a handle to it.
    vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(device_index_);

    // Let's begin setting up the properties now we've got our container.
    // A list of properties available is contained in vr::ETrackedDeviceProperty.
    // Get the ipd of the user from SteamVR settings
    const float ipd = vr::VRSettings()->GetFloat(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_IPD_Float);
    vr::VRProperties()->SetFloatProperty(container, vr::Prop_UserIpdMeters_Float, ipd);

    // For HMDs, it's required that a refresh rate is set otherwise VRCompositor will fail to start.
    vr::VRProperties()->SetFloatProperty(container, vr::Prop_DisplayFrequency_Float, 0.f);

    // The distance from the user's eyes to the display in meters. This is used for reprojection.
    vr::VRProperties()->SetFloatProperty(container, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.f);

    // How long from the compositor to submit a frame to the time it takes to display it on the screen.
    vr::VRProperties()->SetFloatProperty(container, vr::Prop_SecondsFromVsyncToPhotons_Float, 0.11f);

    // avoid "not fullscreen" warnings from vrmonitor
    vr::VRProperties()->SetBoolProperty(container, vr::Prop_IsOnDesktop_Bool, false);

    vr::VRProperties()->SetBoolProperty(container, vr::Prop_DisplayDebugMode_Bool, true);

    // Now let's set up our inputs
    // This tells the UI what to show the user for bindings for this controller,
    // As well as what default bindings should be for legacy apps.
    // Note, we can use the wildcard {<driver_name>} to match the root folder location
    // of our driver.
    vr::VRProperties()->SetStringProperty(container, vr::Prop_InputProfilePath_String, "{simplehmd}/input/mysimplehmd_profile.json");

    // Let's set up handles for all of our components.
    // Even though these are also defined in our input profile,
    // We need to get handles to them to update the inputs.
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/system/touch", &my_input_handles_[MyComponent_system_touch]);
    vr::VRDriverInput()->CreateBooleanComponent(container, "/input/system/click", &my_input_handles_[MyComponent_system_click]);

    // We've activated everything successfully!
    // Let's tell SteamVR that by saying we don't have any errors.
    return vr::VRInitError_None;
}

void* MonitorHMDDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    return nullptr;
}

void MonitorHMDDevice::Deactivate() {
    device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void MonitorHMDDevice::EnterStandby() {

}

//vr::DriverPose_t MonitorHMDDevice::GetPose() {
//    std::lock_guard<std::mutex> lock(pose_mutex_);
//
//    current_pose_.poseIsValid = true;
//    current_pose_.result = vr::TrackingResult_Running_OK;
//    current_pose_.deviceIsConnected = true;
//
//    current_pose_.vecPosition[1] = 1.5;
//
//    current_pose_.qWorldFromDriverRotation.w = 1.0;
//    current_pose_.qWorldFromDriverRotation.x = 0.0;
//    current_pose_.qWorldFromDriverRotation.y = 0.0;
//    current_pose_.qWorldFromDriverRotation.z = 0.0;
//
//    current_pose_.qDriverFromHeadRotation.w = 1.0;
//    current_pose_.qDriverFromHeadRotation.x = 0.0;
//    current_pose_.qDriverFromHeadRotation.y = 0.0;
//    current_pose_.qDriverFromHeadRotation.z = 0.0;
//
//    // Pose is updated by UpdateMouseMovement and potentially keyboard for position
//    current_pose_.qRotation = EulerToQuaternion(hmd_yaw_, hmd_pitch_);
//
//    current_pose_.shouldApplyHeadModel = true;
//
//    memset(current_pose_.vecVelocity, 0, sizeof(current_pose_.vecVelocity));
//    memset(current_pose_.vecAcceleration, 0, sizeof(current_pose_.vecAcceleration));
//    memset(current_pose_.vecAngularVelocity, 0, sizeof(current_pose_.vecAngularVelocity));
//    memset(current_pose_.vecAngularAcceleration, 0, sizeof(current_pose_.vecAngularAcceleration));
//
//    return current_pose_;
//}

vr::DriverPose_t MonitorHMDDevice::GetPose() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    
    vr::TrackedDevicePose_t hmd_pose{};
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, &hmd_pose, 1);

    pose_.vecPosition[1] = sin(frame_number_ * 0.01) * 0.1f + 1.0f;
    pose_.vecPosition[2] = hmd_pose.mDeviceToAbsoluteTracking.m[2][3];

    return pose_;
}

void MonitorHMDDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) {
    if (unResponseBufferSize > 0) pchResponseBuffer[0] = 0;
}

void MonitorHMDDevice::RunFrame() {
    frame_number_++;
    if (is_active_) {
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_index_, GetPose(), sizeof(vr::DriverPose_t));
    }
}

//void MonitorHMDDevice::UpdatePoseThread() {
//    if (is_active_) {
//        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_index_, GetPose(), sizeof(vr::DriverPose_t));
//    }
//}

void MonitorHMDDevice::UpdateMouseMovement(double delta_x, double delta_y) {
    hmd_yaw_ -= delta_x * mouse_sensitivity_;   // Invert X for typical FPS mouse look
    hmd_pitch_ += delta_y * mouse_sensitivity_; // Standard pitch

    // Clamp pitch to +/- 90 degrees (PI/2 radians)
    if (hmd_pitch_ > M_PI / 2.0) hmd_pitch_ = M_PI / 2.0;
    if (hmd_pitch_ < -M_PI / 2.0) hmd_pitch_ = -M_PI / 2.0;

    // Keep yaw between 0 and 2*PI (or -PI to PI)
    hmd_yaw_ = fmod(hmd_yaw_, 2.0 * M_PI);
    if (hmd_yaw_ < 0.0) hmd_yaw_ += 2.0 * M_PI;
}

vr::HmdQuaternion_t MonitorHMDDevice::EulerToQuaternion(double yaw, double pitch) {
    // Assuming Yaw (Y-axis), Pitch (X-axis), Roll (Z-axis, assumed 0 for HMD mouse look)
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    // Assuming roll is 0: cr = 1, sr = 0

    vr::HmdQuaternion_t q;
    q.w = cy * cp;
    q.x = cy * sp; // Pitch around X
    q.y = sy * cp; // Yaw around Y
    q.z = -sy * sp; // Sign can vary depending on convention; this is for YXZ order

    // Normalize (important for precision over time)
    double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm > 0.00001) { // Avoid division by zero
        q.w /= norm; q.x /= norm; q.y /= norm; q.z /= norm;
    }
    else {
        q.w = 1.0; q.x = 0.0; q.y = 0.0; q.z = 0.0;
    }
    return q;
}

