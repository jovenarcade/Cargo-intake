package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class DriveBase extends Subsystem {

    private static DriveBase instance_ = new DriveBase();

    public static DriveBase getInstance() {
        return instance_;
    }

    private final DifferentialDrive mDiffDrive_;
    private final WPI_TalonSRX leftMaster_, leftSlave_, rightMaster_, rightSlave_;
    private final SpeedControllerGroup left_, right_;
    private final ADXRS450_Gyro gyro_;
//    private final ADXL362 accelerometer_;

    private boolean isGyroCalibrating = false;
    private boolean isFacingTargetAngle = false;

    // The constructor instantiates all of the drivetrain components when the
    // robot powers up

    private DriveBase() {
        leftMaster_ = new WPI_TalonSRX(Constants.id_driveLeftMaster);
        leftSlave_ = new WPI_TalonSRX(Constants.id_driveLeftSlave);
        rightMaster_ = new WPI_TalonSRX(Constants.id_driveRightMaster);
        rightSlave_ = new WPI_TalonSRX(Constants.id_driveRightSlave);

        left_ = new SpeedControllerGroup(leftMaster_, leftSlave_);
        right_ = new SpeedControllerGroup(rightMaster_, rightSlave_);

        mDiffDrive_ = new DifferentialDrive(left_, right_);

        // Start in open loop mode
        leftMaster_.set(ControlMode.PercentOutput, 0);
        leftSlave_.follow(leftMaster_);
        rightMaster_.set(ControlMode.PercentOutput, 0);
        rightSlave_.follow(rightMaster_);

//        This is kinda weird but here we are, 2017 robot does not need this
//        rightMaster_.setInverted(true);
//        rightSlave_.setInverted(true);

//        Todo: Determine if setting sensor phase is needed
//        Sets the phase of the sensor. Use when controller forward/reverse output doesn't correlate to appropriate
// forward/reverse reading of sensor. Pick a value so that positive PercentOutput yields a positive change in sensor.
// After setting this, user can freely call SetInverted() with any value.

//        leftMaster_.setSensorPhase(true);
//        rightMaster_.setSensorPhase(true);

        // Establish Drive Encoders
        leftMaster_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rightMaster_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        // Establish Gyro & Accelerometer TODO: Determine if we really need an accelerometer
        gyro_ = new ADXRS450_Gyro(Constants.id_gyroSPIPort);
//        accelerometer_ = new ADXL362();
    }

    public ADXRS450_Gyro getGyro() {
        return gyro_;
    }

    public void calibrateGyro(){
        Timer.delay(0.1);
        isGyroCalibrating = true;
        System.out.println("Gyro Calibration Started. Hold Still...");
        gyro_.calibrate();
        isGyroCalibrating = false;
        System.out.println("Gyro Calibration Finished.");
    }

    public void turnToAngle(double targetAngle){
        double error = targetAngle - gyro_.getAngle();
        double rotation;

        stop();

        if (error > Constants.kGyroTargetAngleThresh) {
            rotation = error * Constants.kGyro_P;
            System.out.println("error" + error);
            System.out.println("Turning to angle " + targetAngle);
        } else {
            rotation = 0;
            isFacingTargetAngle = true;
            System.out.println("Done Turning");
        }

        mDiffDrive_.arcadeDrive(0, rotation);
    }

    public boolean isFacingTargetAngle() {
        return isFacingTargetAngle;
    }

    public DifferentialDrive getDrive(){
        return mDiffDrive_;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("gyro_angle", getGyro().getAngle());
        SmartDashboard.putBoolean("gyro_alive", getGyro().isConnected());
        SmartDashboard.putBoolean("gyro_calibrating", isGyroCalibrating);
        SmartDashboard.putBoolean("gyro_at_target", isFacingTargetAngle);
    }

    @Override
    public void stop() {
        leftMaster_.set(ControlMode.PercentOutput, 0);
        rightMaster_.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zeroSensors() {
        System.out.println("Drive Encoders and Gyro Zeroed.");
        gyro_.reset();
        leftMaster_.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rightMaster_.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }


}
