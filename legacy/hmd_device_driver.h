#pragma once

#include <string>

#include "openvr_driver.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <array>

struct MonitorHMD {
	int32_t window_x = 0;
	int32_t window_y = 0;

	int32_t window_width = 1920;
	int32_t window_height = 1080;

	int32_t render_width = 1920;
	int32_t render_height = 1080;

	float field_of_view_left = -1.0f; // Raw projection values
	float field_of_view_right = 1.0f;
	float field_of_view_top = 1.0f;   // Inverted Y for typical projection
	float field_of_view_bottom = -1.0f;

	float ipd_meters = 0.063f; // Average IPD, less critical if effectively monoscopic
	float display_frequency = 60.0f; // Your monitor's refresh rate
};

enum MyComponent
{
	MyComponent_system_touch,
	MyComponent_system_click,

	MyComponent_MAX
};

class MonitorHMDDisplayComponent : public vr::IVRDisplayComponent {
public:
	explicit MonitorHMDDisplayComponent(const MonitorHMD& config);

	bool IsDisplayOnDesktop() override;
	bool IsDisplayRealDisplay() override;
	void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) override;
	void GetEyeOutputViewport(vr::EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) override;
	void GetProjectionRaw(vr::EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) override;
	vr::DistortionCoordinates_t ComputeDistortion(vr::EVREye eEye, float fU, float fV) override;
	void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) override;
	bool ComputeInverseDistortion(
		vr::HmdVector2_t* pvecOutUndistorted, // Output parameter
		vr::EVREye eEye,
		uint32_t unDistortionFunc,
		float fU, // Input distorted U
		float fV  // Input distorted V
	) override;

private:
	MonitorHMD config_;
};

class MonitorHMDDevice : public vr::ITrackedDeviceServerDriver {
public:
	MonitorHMDDevice();

	// ITrackedDeviceServerDriver
	vr::EVRInitError Activate(uint32_t unObjectId) override;
	void Deactivate() override;
	void EnterStandby() override;
	void* GetComponent(const char* pchComponentNameAndVersion) override;
	void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
	vr::DriverPose_t GetPose() override;
	vr::HmdQuaternion_t EulerToQuaternion(double yaw, double pitch);

	void RunFrame();
	void UpdateMouseMovement(double delta_x, double delta_y);
	std::string GetSerialNumber() const { return "77777777"; };

private:
	std::atomic <uint32_t> device_index_;
	std::string serial_number_;
	std::string model_number_;
	std::atomic <bool> is_active_;
	std::atomic <int> frame_number_;

	std::unique_ptr<MonitorHMDDisplayComponent> display_component_;
	std::array<vr::VRInputComponentHandle_t, MyComponent_MAX> my_input_handles_{};

	//Pose
	vr::DriverPose_t pose_;
	std::mutex pose_mutex_;
	//std::thread pose_update_thread_;
	double hmd_yaw_ = 0.0;   // Radians
	double hmd_pitch_ = 0.0; // Radians
	const double mouse_sensitivity_ = 0.002; // Adjust as needed
};