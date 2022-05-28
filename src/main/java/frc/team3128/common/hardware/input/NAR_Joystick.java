package frc.team3128.common.hardware.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * Wrapper for the WPILib Joystick class. Works with Logitech Extreme 3D Pro and Thrustmaster T16000M.
 * 
 */
public class NAR_Joystick extends Joystick{

    private JoystickButton[] buttons;

    /**
     * POV convention: 0 = up, 45 = top right, 90 = right, 135 = buttom right, 180 = down, 225 = bottom left, 270 = left, 315 = top left
     * We assign indices as angle / 45 [0,7]
     */
    private POVButton[] povButtons;

    private double xDeadband = 0.05;
    private double yDeadband = 0.05;
    private double zDeadband = 0.05;
    private double throttleLowerBound = 0.3;
    private double throttleUpperBound = 0.8;

    public NAR_Joystick(int deviceNumber) {
        super(deviceNumber);
        buttons = new JoystickButton[16];
        povButtons = new POVButton[8];

        // Thrustmaster joystick has 16 buttons
        for (int i = 0; i < 16; i++)
            buttons[i] = new JoystickButton(this, i);
        for (int i = 0; i < 8; i++)
            povButtons[i] = new POVButton(this, i * 45);
    }

    /**  @return Joystick X on [-1, 1], -1 is left, 1 is right - default deadband is 0.05 */
    public double get_X() {
        return Math.abs(getX()) > xDeadband ? getX() : 0;
    }

    /**  @return Joystick Y on [-1, 1], -1 is backwards, 1 is forward - default deadband is 0.05 */
    public double get_Y() {
        return Math.abs(getY()) > yDeadband ? -getY() : 0;
    }

    /**  @return Joystick Z on [-1, 1], -1 is twist left, 1 is twist right - default deadband is 0.05 */
    public double get_Z() {
        return Math.abs(getZ()) > zDeadband ? getZ() : 0;
    }

    /** Alias of getZ */
    public double getTwist() {
        return getZ();
    }

    /**  @return Throttle on [0, 1] where 0 is throttle at bottom, 1 is throttle at top - Default lower bound is 0.3, upper bound is 0.8, so anything below 0.3 returns 0.3, anything above 0.8 returns 1. */
    public double getThrottle() {
        double mappedThrottle = (1 - getThrottle()) / 2;

        if (mappedThrottle < throttleLowerBound)
            return throttleLowerBound;
        else if (mappedThrottle > throttleUpperBound)
            return 1;
        else
            return mappedThrottle;
    }

    public JoystickButton getButton(int i) {
        return buttons[i-1];
    }

    public POVButton getPOVButton(int i) {
        return povButtons[i];
    }

    public POVButton getUpPOVButton() {
        return getPOVButton(0);
    }

    public POVButton getDownPOVButton() {
        return getPOVButton(180);
    }

    public POVButton getLeftPOVButton() {
        return getPOVButton(270);
    }

    public POVButton getRightPOVButton() {
        return getPOVButton(90);
    }

    public void setXDeadband(double xDeadband) {
        this.xDeadband = xDeadband;
    }

    public void setYDeadband(double yDeadband) {
        this.yDeadband = yDeadband;
    }

    public void setZDeadband(double zDeadband) {
        this.zDeadband = zDeadband;
    }
}
