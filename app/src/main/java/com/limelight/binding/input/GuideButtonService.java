package com.limelight.binding.input;

import android.accessibilityservice.AccessibilityService;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.accessibility.AccessibilityEvent;

import com.limelight.Game;

// SHIELD TV turns a gamepad's Mode button (PS, Xbox, etc.) into its own Home action before
// apps get it. Accessibility services see key events earlier, so when the user enables this
// service, Mode presses go straight to the focused stream instead. Outside a stream it
// leaves them alone.
public class GuideButtonService extends AccessibilityService {
    @Override
    protected boolean onKeyEvent(KeyEvent event) {
        if (event.getKeyCode() != KeyEvent.KEYCODE_BUTTON_MODE ||
                (event.getSource() & InputDevice.SOURCE_GAMEPAD) != InputDevice.SOURCE_GAMEPAD) {
            return false;
        }

        Game game = Game.getFocusedInstance();
        if (game == null) {
            return false;
        }

        switch (event.getAction()) {
            case KeyEvent.ACTION_DOWN:
                return game.handleKeyDown(event);
            case KeyEvent.ACTION_UP:
                return game.handleKeyUp(event);
            default:
                return false;
        }
    }

    @Override
    public void onAccessibilityEvent(AccessibilityEvent event) {}

    @Override
    public void onInterrupt() {}
}
