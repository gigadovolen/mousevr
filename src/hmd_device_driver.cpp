#include "hmd_device_driver.h"

#include <string.h>

// Let's create some variables for strings used in getting settings.
// This is the section where all of the settings we want are stored. A section name can be anything,
// but if you want to store driver specific settings, it's best to namespace the section with the driver identifier
// ie "<my_driver>_<section>" to avoid collisions
static const char *my_hmd_main_settings_section = "driver_simplehmd";
static const char *my_hmd_display_settings_section = "simplehmd_display";

MyHMDControllerDeviceDriver::MyHMDControllerDeviceDriver()
{
	// Keep track of whether Activate() has been called
	is_active_ = false;

	// We have our model number and serial number stored in SteamVR settings. We need to get them and do so here.
	// Other IVRSettings methods (to get int32, floats, bools) return the data, instead of modifying, but strings are
	// different.
	char model_number[ 1024 ];
	vr::VRSettings()->GetString( my_hmd_main_settings_section, "model_number", model_number, sizeof( model_number ) );
	my_hmd_model_number_ = model_number;

	// Get our serial number depending on our "handedness"
	char serial_number[ 1024 ];
	vr::VRSettings()->GetString( my_hmd_main_settings_section, "serial_number", serial_number, sizeof( serial_number ) );
	my_hmd_serial_number_ = serial_number;

	// Display settings
	MyHMDDisplayDriverConfiguration display_configuration{};

	display_configuration.window_x = 1920;
	display_configuration.window_y = 1080;
	display_configuration.window_width = 1920;
	display_configuration.window_height = 1080;
	display_configuration.render_width = 1920;
	display_configuration.render_height = 1080;

	// Instantiate our display component
	my_display_component_ = std::make_unique<MyHMDDisplayComponent>(display_configuration);
}

//-----------------------------------------------------------------------------
// Purpose: This is called by vrserver after our
//  IServerTrackedDeviceProvider calls IVRServerDriverHost::TrackedDeviceAdded.
//-----------------------------------------------------------------------------
vr::EVRInitError MyHMDControllerDeviceDriver::Activate( uint32_t unObjectId )
{
	// Let's keep track of our device index. It'll be useful later.
	// Also, if we re-activate, be sure to set this.
	device_index_ = unObjectId;

	// Set a member to keep track of whether we've activated yet or not
	is_active_ = true;

	// For keeping track of frame number for animating motion.
	frame_number_ = 0;

	// Properties are stored in containers, usually one container per device index. We need to get this container to set
	// The properties we want, so we call this to retrieve a handle to it.
	vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer( device_index_ );

	// Let's begin setting up the properties now we've got our container.
	// A list of properties available is contained in vr::ETrackedDeviceProperty.

	// First, let's set the model number.
	vr::VRProperties()->SetStringProperty( container, vr::Prop_ModelNumber_String, my_hmd_model_number_.c_str() );

	// Next, display settings

	// Get the ipd of the user from SteamVR settings
	const float ipd = vr::VRSettings()->GetFloat( vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_IPD_Float );
	vr::VRProperties()->SetFloatProperty( container, vr::Prop_UserIpdMeters_Float, ipd );

	// For HMDs, it's required that a refresh rate is set otherwise VRCompositor will fail to start.
	vr::VRProperties()->SetFloatProperty( container, vr::Prop_DisplayFrequency_Float, 0.f );

	// The distance from the user's eyes to the display in meters. This is used for reprojection.
	vr::VRProperties()->SetFloatProperty( container, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.f );

	// How long from the compositor to submit a frame to the time it takes to display it on the screen.
	vr::VRProperties()->SetFloatProperty( container, vr::Prop_SecondsFromVsyncToPhotons_Float, 0.11f );

	// avoid "not fullscreen" warnings from vrmonitor
	vr::VRProperties()->SetBoolProperty( container, vr::Prop_IsOnDesktop_Bool, false );

	vr::VRProperties()->SetBoolProperty(container, vr::Prop_DisplayDebugMode_Bool, true);

	//vr::VRProperties()->SetStringProperty( container, vr::Prop_InputProfilePath_String, "{mousevr}/resources/input/mysimplehmd_profile.json" );
	//vr::VRDriverInput()->CreateBooleanComponent( container, "/input/system/click", &my_input_handles_[ MyComponent_system_click ] );

	my_pose_update_thread_ = std::thread( &MyHMDControllerDeviceDriver::MyPoseUpdateThread, this );

	// We've activated everything successfully!
	// Let's tell SteamVR that by saying we don't have any errors.
	return vr::VRInitError_None;
}

//-----------------------------------------------------------------------------
// Purpose: If you're an HMD, this is where you would return an implementation
// of vr::IVRDisplayComponent, vr::IVRVirtualDisplay or vr::IVRDirectModeComponent.
//-----------------------------------------------------------------------------
void *MyHMDControllerDeviceDriver::GetComponent( const char *pchComponentNameAndVersion )
{
	if ( strcmp( pchComponentNameAndVersion, vr::IVRDisplayComponent_Version ) == 0 )
	{
		return my_display_component_.get();
	}

	return nullptr;
}

//-----------------------------------------------------------------------------
// Purpose: This is called by vrserver when a debug request has been made from an application to the driver.
// What is in the response and request is up to the application and driver to figure out themselves.
//-----------------------------------------------------------------------------
void MyHMDControllerDeviceDriver::DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize )
{
	if ( unResponseBufferSize >= 1 )
		pchResponseBuffer[ 0 ] = 0;
}

//-----------------------------------------------------------------------------
// Purpose: This is never called by vrserver in recent OpenVR versions,
// but is useful for giving data to vr::VRServerDriverHost::TrackedDevicePoseUpdated.
//-----------------------------------------------------------------------------
vr::DriverPose_t MyHMDControllerDeviceDriver::GetPose() {
	vr::DriverPose_t pose{ 0 };

	pose.qWorldFromDriverRotation.w = 1.f;
	pose.qDriverFromHeadRotation.w = 1.f;

	pose.vecPosition[0] = 0.0f;
	pose.vecPosition[1] = 1.8f;
	pose.vecPosition[2] = 0.0f;

	pose.qRotation = EulerToQuaternion(hmd_yaw_, hmd_pitch_);

	pose.poseIsValid = true;
	pose.deviceIsConnected = true;

	pose.result = vr::TrackingResult_Running_OK;
	pose.shouldApplyHeadModel = false;

	return pose;
}

void MyHMDControllerDeviceDriver::MyPoseUpdateThread()
{
	while ( is_active_ )
	{
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated( device_index_, GetPose(), sizeof( vr::DriverPose_t ) );
	}
}

void MyHMDControllerDeviceDriver::UpdateMouseMovement(double delta_x, double delta_y) {
	hmd_yaw_ -= delta_x * mouse_sensitivity_;
	hmd_pitch_ += delta_y * mouse_sensitivity_;

	if (hmd_pitch_ > M_PI / 2.0) hmd_pitch_ = M_PI / 2.0;
	if (hmd_pitch_ < -M_PI / 2.0) hmd_pitch_ = -M_PI / 2.0;

	hmd_yaw_ = fmod(hmd_yaw_, 2.0 * M_PI);
	if (hmd_yaw_ < 0.0) hmd_yaw_ += 2.0 * M_PI;
}

//-----------------------------------------------------------------------------
// Purpose: This is called by vrserver when the device should enter standby mode.
// The device should be put into whatever low power mode it has.
// We don't really have anything to do here, so let's just log something.
//-----------------------------------------------------------------------------
void MyHMDControllerDeviceDriver::EnterStandby()
{
}

//-----------------------------------------------------------------------------
// Purpose: This is called by vrserver when the device should deactivate.
// This is typically at the end of a session
// The device should free any resources it has allocated here.
//-----------------------------------------------------------------------------
void MyHMDControllerDeviceDriver::Deactivate()
{
	// Let's join our pose thread that's running
	// by first checking then setting is_active_ to false to break out
	// of the while loop, if it's running, then call .join() on the thread
	if ( is_active_.exchange( false ) )
	{
		my_pose_update_thread_.join();
	}

	// unassign our controller index (we don't want to be calling vrserver anymore after Deactivate() has been called
	device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}


//-----------------------------------------------------------------------------
// Purpose: This is called by our IServerTrackedDeviceProvider when its RunFrame() method gets called.
// It's not part of the ITrackedDeviceServerDriver interface, we created it ourselves.
//-----------------------------------------------------------------------------
void MyHMDControllerDeviceDriver::MyRunFrame()
{
	frame_number_++;
}


//-----------------------------------------------------------------------------
// Purpose: This is called by our IServerTrackedDeviceProvider when it pops an event off the event queue.
// It's not part of the ITrackedDeviceServerDriver interface, we created it ourselves.
//-----------------------------------------------------------------------------
void MyHMDControllerDeviceDriver::MyProcessEvent( const vr::VREvent_t &vrevent )
{
}


//-----------------------------------------------------------------------------
// Purpose: Our IServerTrackedDeviceProvider needs our serial number to add us to vrserver.
// It's not part of the ITrackedDeviceServerDriver interface, we created it ourselves.
//-----------------------------------------------------------------------------
const std::string &MyHMDControllerDeviceDriver::MyGetSerialNumber()
{
	return my_hmd_serial_number_;
}

vr::HmdQuaternion_t MyHMDControllerDeviceDriver::EulerToQuaternion(double yaw, double pitch) {
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

//-----------------------------------------------------------------------------
// DISPLAY DRIVER METHOD DEFINITIONS
//-----------------------------------------------------------------------------

MyHMDDisplayComponent::MyHMDDisplayComponent( const MyHMDDisplayDriverConfiguration &config )
	: config_( config )
{
}

//-----------------------------------------------------------------------------
// Purpose: To inform vrcompositor if this display is considered an on-desktop display.
//-----------------------------------------------------------------------------
bool MyHMDDisplayComponent::IsDisplayOnDesktop()
{
	return true;
}

//-----------------------------------------------------------------------------
// Purpose: To as vrcompositor to search for this display.
//-----------------------------------------------------------------------------
bool MyHMDDisplayComponent::IsDisplayRealDisplay()
{
	return false;  //ALWAYS FALSE DOES NOT WORK OTHERWISE
}

bool MyHMDDisplayComponent::ComputeInverseDistortion(
	vr::HmdVector2_t* pvecOutUndistorted, // Output parameter
	vr::EVREye eEye,
	uint32_t unDistortionFunc,
	float fU, // Input distorted U
	float fV  // Input distorted V
) {
	return false;
}

//-----------------------------------------------------------------------------
// Purpose: To inform the rest of the vr system what the recommended target size should be
//-----------------------------------------------------------------------------
void MyHMDDisplayComponent::GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnWidth = config_.render_width;
	*pnHeight = config_.render_height;
}

//-----------------------------------------------------------------------------
// Purpose: To inform vrcompositor how the screens should be organized.
//-----------------------------------------------------------------------------
void MyHMDDisplayComponent::GetEyeOutputViewport( vr::EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnY = 0;

	// Each eye will have the full height
	*pnHeight = config_.window_height;

	if ( eEye == vr::Eye_Left )
	{
		*pnX = 0;
		*pnWidth = config_.window_width;
	}
	else
	{
		// Right eye NO VIEWPORT
		*pnX = config_.window_width;
		*pnWidth = 0;
	}
}

//-----------------------------------------------------------------------------
// Purpose: To inform the compositor what the projection parameters are for this HMD.
//-----------------------------------------------------------------------------
void MyHMDDisplayComponent::GetProjectionRaw( vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom )
{
	float aspect_ratio = static_cast<float>(config_.render_width) / static_cast<float>(config_.render_height);

	float top = -1.0f;
	float bottom = 1.0f;

	*pfLeft = top * aspect_ratio;
	*pfRight = bottom * aspect_ratio;
	*pfTop = top;
	*pfBottom = bottom;
}

//-----------------------------------------------------------------------------
// Purpose: To compute the distortion properties for a given uv in an image.
//-----------------------------------------------------------------------------
vr::DistortionCoordinates_t MyHMDDisplayComponent::ComputeDistortion( vr::EVREye eEye, float fU, float fV )
{
	vr::DistortionCoordinates_t coordinates{};
	coordinates.rfBlue[ 0 ] = fU;
	coordinates.rfBlue[ 1 ] = fV;
	coordinates.rfGreen[ 0 ] = fU;
	coordinates.rfGreen[ 1 ] = fV;
	coordinates.rfRed[ 0 ] = fU;
	coordinates.rfRed[ 1 ] = fV;
	return coordinates;
}

//-----------------------------------------------------------------------------
// Purpose: To inform vrcompositor what the window bounds for this virtual HMD are.
//-----------------------------------------------------------------------------
void MyHMDDisplayComponent::GetWindowBounds( int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
{
	*pnX = config_.window_x;
	*pnY = config_.window_y;
	*pnWidth = config_.window_width;
	*pnHeight = config_.window_height;
}
