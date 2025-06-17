#include "device_provider.h"
#if defined(_WIN32)
#include "windows.h"
#endif

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext* pDriverContext) {
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    my_left_device_ = std::make_unique<ControllerDevice>(vr::TrackedControllerRole_LeftHand);
    vr::VRServerDriverHost()->TrackedDeviceAdded("<keyboard>",
        vr::TrackedDeviceClass_Controller,
        my_left_device_.get());

    my_hmd_device_ = std::make_unique<MyHMDControllerDeviceDriver>();
    if (my_hmd_device_) {
        vr::VRServerDriverHost()->TrackedDeviceAdded(
            my_hmd_device_->GetSerialNumber().c_str(),
            vr::TrackedDeviceClass_HMD,
            my_hmd_device_.get());
    }

    first_mouse_update_ = true;

    return vr::VRInitError_None;
}

void DeviceProvider::Cleanup() {
    my_hmd_device_.reset();
    my_left_device_.reset();
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

const char* const* DeviceProvider::GetInterfaceVersions() {
    return vr::k_InterfaceVersions;
}

void DeviceProvider::RunFrame() {
    if (my_left_device_ != nullptr) {
#if defined(_WIN32)
        Inputs currFrame{0};

        currFrame.ESCPressed   = (GetAsyncKeyState(VK_ESCAPE) & 0x8000) != 0;
        currFrame.SPACEPressed = (GetAsyncKeyState(VK_SPACE) & 0x8000) != 0;
        currFrame.wPressed     = (GetAsyncKeyState('W') & 0x8000) != 0;
        currFrame.sPressed     = (GetAsyncKeyState('S') & 0x8000) != 0;
        currFrame.aPressed     = (GetAsyncKeyState('A') & 0x8000) != 0;
        currFrame.dPressed     = (GetAsyncKeyState('D') & 0x8000) != 0;
        currFrame.zPressed     = (GetAsyncKeyState('Z') & 0x8000) != 0;
        currFrame.vPressed     = (GetAsyncKeyState('V') & 0x8000) != 0;
        currFrame.yPressed     = (GetAsyncKeyState('Y') & 0x8000) != 0;
        currFrame.rPressed     = (GetAsyncKeyState('R') & 0x8000) != 0;
        currFrame.cPressed     = (GetAsyncKeyState('C') & 0x8000) != 0;
        currFrame.fPressed     = (GetAsyncKeyState('F') & 0x8000) != 0;
        currFrame.qPressed     = (GetAsyncKeyState('Q') & 0x8000) != 0;
        currFrame.ePressed     = (GetAsyncKeyState('E') & 0x8000) != 0;
        currFrame.oPressed     = (GetAsyncKeyState('O') & 0x8000) != 0;
        currFrame.pPressed     = (GetAsyncKeyState('P') & 0x8000) != 0;
#endif
        my_left_device_->RunFrame(currFrame);
    }

    if (my_hmd_device_) {
#if defined(_WIN32)
        POINT cursorPos;
        if (GetCursorPos(&cursorPos)) {
            if (first_mouse_update_) {
                prev_mouse_x_ = cursorPos.x;
                prev_mouse_y_ = cursorPos.y;
                first_mouse_update_ = false;
            }
            else {
                double delta_x = static_cast<double>(cursorPos.x - prev_mouse_x_);
                double delta_y = static_cast<double>(prev_mouse_y_ - cursorPos.y);

                my_hmd_device_->UpdateMouseMovement(delta_x, delta_y);

                prev_mouse_x_ = cursorPos.x;
                prev_mouse_y_ = cursorPos.y;
            }
        }
#endif
        my_hmd_device_->MyRunFrame();
    }
}

bool DeviceProvider::ShouldBlockStandbyMode() {
    return true;
}

void DeviceProvider::EnterStandby() {

}

void DeviceProvider::LeaveStandby() {

}