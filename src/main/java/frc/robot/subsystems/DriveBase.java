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
import frc.robot.util.Utils;

import static frc.robot.subsystems.DriveBase.AssistMode.*;

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
    private double rotation;
    private double throttle;
    private double deltaAngle;

    public enum AssistMode {
        NONE,
        HEADING,
        LINEAR
    }
    AssistMode mAssistMode_ = NONE;

    private PIDController headingPIDController_;

    private ControlMode driveMode = ControlMode.RAW;

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

        leftSlave_.follow(leftMaster_);
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

        setTalonControlMode(driveMode);


        //Set up a PIDController for controlling robot's heading using mGyro as a feedback sensor
        headingPIDController_ = new PIDController(
                Constants.kHeadingClosedLoop_P, Constants.kHeadingClosedLoop_I, Constants.kHeadingClosedLoop_D,
                getGyro(),
                new PIDOutput() {
                    @Override
                    public void pidWrite(double output) {
                        //Turn robot however much the PID controller tells us to
                        //RobotDrive and Gyro have different directions set to + TODO: I dont think this is still true
//                        mDiffDrive_.arcadeDrive(0, -output);
                        rotation = output;
                    }
                });
        //Robot can turn either way to reach setpoint; do whichever is shortest
        headingPIDController_.setInputRange(-180, 180);
//        headingPIDController_.setOutputRange(-10, 10);
        headingPIDController_.setContinuous(true);
        //TODO What's our min and max? setContinuous is useless without one. - NO IDEA IF THESE ARE EVEN REMOTELY RIGHT
        //Output range doesn't seem to make a difference, or im not using it right, but the input of -180 180 is great
        //for turning shortest distance possible
        // tested on 2017 robot on concrete in LC - 01262019
    }

    public void calibrateGyro(){
        Timer.delay(0.1);
        isGyroCalibrating = true;
        System.out.println("Gyro Calibration Started. Hold Still...");
        gyro_.calibrate();
        isGyroCalibrating = false;
        System.out.println("Gyro Calibration Finished.");
    }

    public double getGyroPIDOutput(){
        return rotation;
    }

    public DifferentialDrive getDrive(){
        setTalonControlMode(ControlMode.RAW);
        return mDiffDrive_;
    }

//    If the similar design works in teleopPeriodic, try it here
    public void getGyroDrive(double throttle, double relativeSetpoint){
        setAssistMode(DriveBase.AssistMode.HEADING);
        setRelativeSetpoint(relativeSetpoint);
        getDrive().arcadeDrive(throttle, getGyroPIDOutput());
    }

    public enum ControlMode{
        RAW, ENCODER_VEL, ENCODER_POS
    }

    public void setTalonControlMode(ControlMode mode){
        switch (mode){
            case RAW:
                rightMaster_.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
                leftMaster_.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
                break;

            case ENCODER_VEL:
                rightMaster_.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, 0);
                leftMaster_.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, 0);
                break;

            case ENCODER_POS:
                rightMaster_.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, 0);
                leftMaster_.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, 0);
                break;
        }
    }


    /**
     * Sets TalonControlMode. Use this to switch between normal drive mode, heading, and linear setpoints.
     * @param assistMode NONE, HEADING, LINEAR
     */
    public void setAssistMode(AssistMode assistMode){
        this.mAssistMode_ = assistMode;
        switch (assistMode){
            case NONE:
                setTalonControlMode(ControlMode.RAW);
                headingPIDController_.disable();
                break;
            case HEADING:
                setTalonControlMode(ControlMode.RAW);
                headingPIDController_.enable();
                System.out.println("Gyro PID Out: " + rotation);
                break;
            case LINEAR:
                setTalonControlMode(ControlMode.ENCODER_POS);
                headingPIDController_.disable();
                break;
        }
    }
    /**
     * Set HEADING or LINEAR setpoint, depending on TribeRobotDrive's current assistMode
     */
    public void setSetpoint(double setpoint){
        switch (mAssistMode_){
            case NONE:
                //TODO Shouldn't be called like this. Should we throw an error?
                break;
            case HEADING:
                headingPIDController_.setSetpoint(setpoint);
                break;
            case LINEAR:
                setSetpointRight(setpoint);
                setSetpointLeft(setpoint);
                break;
        }
    }

    /**
     * Calls setSetpoint(deltaSetpoint + currentPosition)
     */
    public void setRelativeSetpoint(double deltaSetpoint){
        switch (mAssistMode_){
            case NONE:
                //TODO Shouldn't be called like this. Should we throw an error?
                break;
            case HEADING:
                headingPIDController_.setSetpoint(gyro_.getAngle() + deltaSetpoint);
                break;
            case LINEAR:
                setSetpointRight(rightMaster_.getSelectedSensorPosition() + deltaSetpoint);
                setSetpointLeft(leftMaster_.getSelectedSensorPosition() + deltaSetpoint);
                break;
        }
    }

    /**
     *
     * @return current setpoint of either the heading or linear, depending on assistMode
     */
    public double getSetpoint(){
        switch (mAssistMode_){
            case NONE:
                //TODO Shouldn't be called like this. Should we throw an error?
                System.out.println("No setpnt b/c AssistMode is NONE");
                return 0;
            case HEADING:
                return headingPIDController_.getSetpoint();
            case LINEAR:
                return (getSetpointRight() + getSetpointLeft()) / 2;
            default:
                return 0;
        }

    }

    /**
     *
     * @return current error of either the heading or linear, depending on assistMode
     */
    public double getSetpointError(){
        switch (mAssistMode_){
            case NONE:
                return 0;
            case HEADING:
                return headingPIDController_.getError();
            case LINEAR:
                return (rightMaster_.getClosedLoopError() + leftMaster_.getClosedLoopError()) / 2;
            default:
                return 0;
        }
    }

    /**
     *
     * @return If TribeRobotDrive's error is less than the tolerance
     */
    public boolean onTarget(){
        switch (mAssistMode_){
            case NONE:
                return false;
            case HEADING:
                return headingPIDController_.getError() < Constants.kGyroTargetAngleThresh;
            case LINEAR:
                System.out.println(rightMaster_.getClosedLoopError());
                return Math.abs(((rightMaster_.getClosedLoopError() + leftMaster_.getClosedLoopError()) / 2)) <= Constants.kLinearClosedLoop_Tolerance_Default;
            default:
                return false;
        }
    }

    public void setSetpointRight(double setpoint){
        rightMaster_.set(setpoint);
    }
    public void setSetpointLeft(double setpoint){
        leftMaster_.set(setpoint);
    }
    public double getSetpointRight(){
        return rightMaster_.getClosedLoopTarget(Constants.kPIDLoopIdx);
    }
    public double getSetpointLeft(){
        return leftMaster_.getClosedLoopTarget(Constants.kPIDLoopIdx);
    }


    public ADXRS450_Gyro getGyro(){
        return gyro_;
    }


    public double getLeftDistanceInches(){
        return Utils.encUnitsToInches(leftMaster_.getSelectedSensorPosition());
    }

    public double getRightDistanceInches(){
        return Utils.encUnitsToInches(rightMaster_.getSelectedSensorPosition());
    }

    public double getLeftVelocityInchesPerSecond(){
        return Utils.encVelToInchesPerSecond(leftMaster_.getSelectedSensorVelocity());
    }

    public double getRightVelocityInchesPerSecond(){
        return Utils.encVelToInchesPerSecond(rightMaster_.getSelectedSensorVelocity());
    }


    /**
     * Grabs PID heading data from SmartDashboard. Used for debugging robot.
     */
    public void setPIDFromSmartDashboard(){
        headingPIDController_.setPID(
                SmartDashboard.getNumber("headingPID_P", Constants.kHeadingClosedLoop_P),
                SmartDashboard.getNumber("headingPID_I", Constants.kHeadingClosedLoop_I),
                SmartDashboard.getNumber("headingPID_D", Constants.kHeadingClosedLoop_D));
    }

    /**
     * Prints status to SmartDashboard
     */
    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("gyro_angle", getGyro().getAngle());
        SmartDashboard.putBoolean("gyro_alive", getGyro().isConnected());
        SmartDashboard.putBoolean("gyro_calibrating", isGyroCalibrating);
        SmartDashboard.putBoolean("gyro_at_target", isFacingTargetAngle);

        SmartDashboard.putNumber("left_distance", getLeftDistanceInches());
        SmartDashboard.putNumber("right_distance", getRightDistanceInches());
        SmartDashboard.putNumber("left_velocity", getLeftVelocityInchesPerSecond());
        SmartDashboard.putNumber("right_velocity", getRightVelocityInchesPerSecond());
        SmartDashboard.putNumber("left_error", leftMaster_.getClosedLoopError());
        SmartDashboard.putNumber("right_error", rightMaster_.getClosedLoopError());

        SmartDashboard.putNumber("robotdrive_setpoint_error", getSetpointError());

        //Set up P I and D parameters in SmartDashboard
//        SmartDashboard.putNumber("headingPID_P", Constants.kHeadingClosedLoop_P);
//        SmartDashboard.putNumber("headingPID_I", Constants.kHeadingClosedLoop_I);
//        SmartDashboard.putNumber("headingPID_D", Constants.kHeadingClosedLoop_D);
    }

    @Override
    public void stop() {
        leftMaster_.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
        rightMaster_.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
    }

    /**
     * Resets and zeros all sensors
     */
    @Override
    public void zeroSensors() {
        System.out.println("Drive Encoders and Gyro Zeroed.");
        gyro_.reset();
        leftMaster_.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rightMaster_.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    }


}
