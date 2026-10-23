package com.limelight.binding.input.driver;

import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.os.SystemClock;
import android.view.InputDevice;

import com.limelight.LimeLog;
import com.limelight.nvstream.input.ControllerPacket;
import com.limelight.nvstream.jni.MoonBridge;

// DualSense and DualSense Edge over USB. Kernels without the PS5 controller driver (hid-playstation,
// Linux 5.12+) expose these as generic gamepads with no rumble, motion sensors, touchpad or Edge
// paddles, so on those devices we talk to the controller's HID interface directly.
// Report layouts follow drivers/hid/hid-playstation.c.
public class DualSenseController extends AbstractController {
    private static final int SONY_VID = 0x054c;
    private static final int DUALSENSE_PID = 0x0ce6;
    private static final int DUALSENSE_EDGE_PID = 0x0df2;

    private static final int INPUT_REPORT_USB = 0x01;
    private static final int OUTPUT_REPORT_USB = 0x02;
    private static final int OUTPUT_REPORT_USB_SIZE = 63;

    // Output report valid_flag bits
    private static final int FLAG0_COMPATIBLE_VIBRATION = 0x01;
    private static final int FLAG0_HAPTICS_SELECT = 0x02;
    private static final int FLAG2_COMPATIBLE_VIBRATION2 = 0x04;

    private final UsbDevice device;
    private final UsbDeviceConnection connection;

    private UsbInterface hidIface;
    private UsbEndpoint inEndpt, outEndpt;

    private Thread inputThread;
    private volatile boolean stopped;

    // Sticks, triggers and buttons from the last report (bytes 1-6 and 8-11). The controller
    // reports continuously, so we only pass input on when one of these changes.
    private final byte[] lastState = new byte[10];

    public static boolean canClaimDevice(UsbDevice device) {
        return device.getVendorId() == SONY_VID &&
                (device.getProductId() == DUALSENSE_PID || device.getProductId() == DUALSENSE_EDGE_PID);
    }

    // The kernel's PS5 driver gives the controller a vibrator, so if Android reports one
    // we can leave the controller to the built-in support
    public static boolean kernelSupportsDevice(UsbDevice device) {
        for (int id : InputDevice.getDeviceIds()) {
            InputDevice inputDev = InputDevice.getDevice(id);
            if (inputDev != null &&
                    inputDev.getVendorId() == device.getVendorId() &&
                    inputDev.getProductId() == device.getProductId() &&
                    inputDev.getVibrator().hasVibrator()) {
                return true;
            }
        }

        return false;
    }

    public DualSenseController(UsbDevice device, UsbDeviceConnection connection, int deviceId, UsbDriverListener listener) {
        super(deviceId, listener, device.getVendorId(), device.getProductId());
        this.device = device;
        this.connection = connection;
        this.type = MoonBridge.LI_CTYPE_PS;
        this.capabilities = MoonBridge.LI_CCAP_ANALOG_TRIGGERS | MoonBridge.LI_CCAP_RUMBLE;
        this.supportedButtonFlags =
                ControllerPacket.A_FLAG | ControllerPacket.B_FLAG | ControllerPacket.X_FLAG | ControllerPacket.Y_FLAG |
                        ControllerPacket.UP_FLAG | ControllerPacket.DOWN_FLAG | ControllerPacket.LEFT_FLAG | ControllerPacket.RIGHT_FLAG |
                        ControllerPacket.LB_FLAG | ControllerPacket.RB_FLAG |
                        ControllerPacket.LS_CLK_FLAG | ControllerPacket.RS_CLK_FLAG |
                        ControllerPacket.BACK_FLAG | ControllerPacket.PLAY_FLAG | ControllerPacket.SPECIAL_BUTTON_FLAG |
                        ControllerPacket.TOUCHPAD_FLAG | ControllerPacket.MISC_FLAG;
        if (device.getProductId() == DUALSENSE_EDGE_PID) {
            this.supportedButtonFlags |=
                    ControllerPacket.PADDLE1_FLAG | ControllerPacket.PADDLE2_FLAG |
                            ControllerPacket.PADDLE3_FLAG | ControllerPacket.PADDLE4_FLAG;
        }
    }

    private Thread createInputThread() {
        return new Thread() {
            public void run() {
                try {
                    // Give the old InputDevice time to go away so we can take its controller number
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    return;
                }

                notifyDeviceAdded();

                byte[] buffer = new byte[64];
                while (!isInterrupted() && !stopped) {
                    // A failure well before the timeout means the device is gone
                    long lastMillis = SystemClock.uptimeMillis();
                    int res = connection.bulkTransfer(inEndpt, buffer, buffer.length, 3000);
                    if (res <= 0) {
                        if (SystemClock.uptimeMillis() - lastMillis < 1000) {
                            LimeLog.warning("DualSense: device I/O error");
                            DualSenseController.this.stop();
                            break;
                        }
                        continue;
                    }

                    if (handleRead(buffer, res)) {
                        reportInput();
                    }
                }
            }
        };
    }

    private boolean handleRead(byte[] report, int length) {
        if (report[0] != INPUT_REPORT_USB || length < 12) {
            return false;
        }

        // Skip the sequence number at byte 7
        boolean changed = false;
        for (int i = 0; i < lastState.length; i++) {
            byte b = report[i < 6 ? i + 1 : i + 2];
            if (lastState[i] != b) {
                lastState[i] = b;
                changed = true;
            }
        }
        if (!changed) {
            return false;
        }

        leftStickX = stickValue(report[1]);
        leftStickY = stickValue(report[2]);
        rightStickX = stickValue(report[3]);
        rightStickY = stickValue(report[4]);
        leftTrigger = (report[5] & 0xFF) / 255.0f;
        rightTrigger = (report[6] & 0xFF) / 255.0f;

        int buttons0 = report[8] & 0xFF;
        int buttons1 = report[9] & 0xFF;
        int buttons2 = report[10] & 0xFF;

        // D-pad is a hat: 0 = up, then clockwise in 45 degree steps, 8 = released
        int hat = buttons0 & 0x0F;
        setButtonFlag(ControllerPacket.UP_FLAG, (hat == 7 || hat == 0 || hat == 1) ? 1 : 0);
        setButtonFlag(ControllerPacket.RIGHT_FLAG, (hat >= 1 && hat <= 3) ? 1 : 0);
        setButtonFlag(ControllerPacket.DOWN_FLAG, (hat >= 3 && hat <= 5) ? 1 : 0);
        setButtonFlag(ControllerPacket.LEFT_FLAG, (hat >= 5 && hat <= 7) ? 1 : 0);

        setButtonFlag(ControllerPacket.X_FLAG, buttons0 & 0x10); // Square
        setButtonFlag(ControllerPacket.A_FLAG, buttons0 & 0x20); // Cross
        setButtonFlag(ControllerPacket.B_FLAG, buttons0 & 0x40); // Circle
        setButtonFlag(ControllerPacket.Y_FLAG, buttons0 & 0x80); // Triangle

        setButtonFlag(ControllerPacket.LB_FLAG, buttons1 & 0x01);
        setButtonFlag(ControllerPacket.RB_FLAG, buttons1 & 0x02);
        setButtonFlag(ControllerPacket.BACK_FLAG, buttons1 & 0x10); // Create
        setButtonFlag(ControllerPacket.PLAY_FLAG, buttons1 & 0x20); // Options
        setButtonFlag(ControllerPacket.LS_CLK_FLAG, buttons1 & 0x40);
        setButtonFlag(ControllerPacket.RS_CLK_FLAG, buttons1 & 0x80);

        setButtonFlag(ControllerPacket.SPECIAL_BUTTON_FLAG, buttons2 & 0x01); // PS
        setButtonFlag(ControllerPacket.TOUCHPAD_FLAG, buttons2 & 0x02);
        setButtonFlag(ControllerPacket.MISC_FLAG, buttons2 & 0x04); // Mute

        if (device.getProductId() == DUALSENSE_EDGE_PID) {
            setButtonFlag(ControllerPacket.PADDLE4_FLAG, buttons2 & 0x10); // Left Fn
            setButtonFlag(ControllerPacket.PADDLE3_FLAG, buttons2 & 0x20); // Right Fn
            setButtonFlag(ControllerPacket.PADDLE2_FLAG, buttons2 & 0x40); // Left back button
            setButtonFlag(ControllerPacket.PADDLE1_FLAG, buttons2 & 0x80); // Right back button
        }

        return true;
    }

    // 0-255 with 128 centered, up/left at 0
    private static float stickValue(byte raw) {
        return Math.max(-1.0f, ((raw & 0xFF) - 128) / 127.0f);
    }

    public boolean start() {
        for (int i = 0; i < device.getInterfaceCount(); i++) {
            UsbInterface iface = device.getInterface(i);
            if (iface.getInterfaceClass() == UsbConstants.USB_CLASS_HID) {
                hidIface = iface;
                break;
            }
        }
        if (hidIface == null) {
            LimeLog.warning("DualSense: no HID interface");
            return false;
        }

        // Only the HID interface: the audio interfaces stay with Android
        if (!connection.claimInterface(hidIface, true)) {
            LimeLog.warning("DualSense: failed to claim HID interface");
            return false;
        }

        for (int i = 0; i < hidIface.getEndpointCount(); i++) {
            UsbEndpoint endpt = hidIface.getEndpoint(i);
            if (endpt.getType() != UsbConstants.USB_ENDPOINT_XFER_INT) {
                continue;
            }

            if (endpt.getDirection() == UsbConstants.USB_DIR_IN) {
                inEndpt = endpt;
            }
            else {
                outEndpt = endpt;
            }
        }
        if (inEndpt == null || outEndpt == null) {
            LimeLog.warning("DualSense: missing interrupt endpoint");
            connection.releaseInterface(hidIface);
            return false;
        }

        inputThread = createInputThread();
        inputThread.start();

        return true;
    }

    public void stop() {
        if (stopped) {
            return;
        }

        stopped = true;

        rumble((short)0, (short)0);

        if (inputThread != null) {
            inputThread.interrupt();
            inputThread = null;
        }

        // Releasing the interface lets the kernel driver take the controller back
        if (hidIface != null) {
            connection.releaseInterface(hidIface);
        }
        connection.close();

        notifyDeviceRemoved();
    }

    @Override
    public void rumble(short lowFreqMotor, short highFreqMotor) {
        if (outEndpt == null) {
            return;
        }

        byte[] report = new byte[OUTPUT_REPORT_USB_SIZE];
        report[0] = OUTPUT_REPORT_USB;

        // Classic rumble emulation. Newer firmware (and the Edge) reads the second
        // compatible-vibration flag instead, so set both like SDL does.
        report[1] = FLAG0_COMPATIBLE_VIBRATION | FLAG0_HAPTICS_SELECT;
        report[3] = (byte)((highFreqMotor >> 8) & 0xFF); // motor_right
        report[4] = (byte)((lowFreqMotor >> 8) & 0xFF); // motor_left
        report[39] = FLAG2_COMPATIBLE_VIBRATION2;

        int res = connection.bulkTransfer(outEndpt, report, report.length, 100);
        if (res != report.length) {
            LimeLog.warning("DualSense: rumble write failed: "+res);
        }
    }

    @Override
    public void rumbleTriggers(short leftTrigger, short rightTrigger) {
        // No trigger motors; adaptive trigger effects aren't part of the stream
    }
}
