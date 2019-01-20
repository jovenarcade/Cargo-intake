package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.JoystickConstants;

/**
 * A basic framework for the control board Like the drive code, one instance of
 * the ControlBoard object is created upon startup, then other methods request
 * the singleton ControlBoard instance.
 */

public class Controllers {

    private static Controllers mInstance = new Controllers();

    public static Controllers getInstance() {
        return mInstance;
    }

    private final Joystick mDriveStick; //20190116 - an XBOX Controller
    private final Joystick mAuxStick;

    private Controllers() {
        mDriveStick = new Joystick(0);
        mAuxStick = new Joystick(1);
    }

    //Driver Joystick controls
    public double getThrottle() {
        return -mDriveStick.getRawAxis(JoystickConstants.kXBOX_LJoyY);
    }

    public double getTurn() {
        return mDriveStick.getRawAxis(JoystickConstants.kXBOX_RJoyX);
    }

    public boolean driveStraightWithGyro() {
        return mDriveStick.getRawButton(JoystickConstants.kF310_LBump);
    }

    public boolean getValuesFromSmartDashboard() {
        return mDriveStick.getRawButton(JoystickConstants.kF310_Start);
    }

    public boolean spinGyroToAngle() {
        return mDriveStick.getRawButton(JoystickConstants.kF310_RBump);
    }

    public void setVibrateSeconds(int vibrateSeconds){
        mDriveStick.setRumble(GenericHID.RumbleType.kRightRumble, 1);
        mDriveStick.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        Timer.delay(vibrateSeconds);
    }
}
