package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
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

//    private final DifferentialDrive mDiffDrive_;
    private final TalonSRX leftMaster_, leftSlave_, rightMaster_, rightSlave_;
//    private final SpeedControllerGroup left_, right_;
    private final ADXRS450_Gyro gyro_;

    // The constructor instantiates all of the drivetrain components when the
    // robot powers up

    private DriveBase() {
        leftMaster_ = new TalonSRX(Constants.id_driveLeftMaster);
        leftSlave_ = new TalonSRX(Constants.id_driveLeftSlave);
        rightMaster_ = new TalonSRX(Constants.id_driveRightMaster);
        rightSlave_ = new TalonSRX(Constants.id_driveRightSlave);

//        left_ = new SpeedControllerGroup(leftMaster_, leftSlave_);
//        right_ = new SpeedControllerGroup(rightMaster_, rightSlave_);

//        mDiffDrive_ = new DifferentialDrive(left_, right_);

        gyro_ = new ADXRS450_Gyro(Constants.id_gyroSPIPort);

        // Start in open loop mode
        leftMaster_.set(ControlMode.PercentOutput, 0);
        leftSlave_.follow(leftMaster_);
        rightMaster_.set(ControlMode.PercentOutput, 0);
        rightSlave_.follow(rightMaster_);
        rightMaster_.setInverted(true);
    }

    public ADXRS450_Gyro getGyro() {
        return gyro_;
    }

//    public DifferentialDrive getmDiffDrive_(){
//        return mDiffDrive_;
//    }

    public void gyroSpinTest(int desiredAngle){

    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("gyro_angle", getGyro().getAngle());
        SmartDashboard.putBoolean("gyro_alive", getGyro().isConnected());
    }

    @Override
    public void stop() {
        leftMaster_.set(ControlMode.PercentOutput, 0);
        rightMaster_.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zeroSensors() {
        gyro_.reset();
    }


}
