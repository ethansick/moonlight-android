package com.limelight.binding.input.driver;

public interface UsbDriverListener {
    void reportControllerState(int controllerId, int buttonFlags,
                               float leftStickX, float leftStickY,
                               float rightStickX, float rightStickY,
                               float leftTrigger, float rightTrigger);

    // Gyro in deg/s or accelerometer in m/s^2 (motionType is a MoonBridge.LI_MOTION_TYPE_* value)
    void reportControllerMotion(int controllerId, byte motionType, float x, float y, float z);

    void deviceRemoved(AbstractController controller);
    void deviceAdded(AbstractController controller);
}
