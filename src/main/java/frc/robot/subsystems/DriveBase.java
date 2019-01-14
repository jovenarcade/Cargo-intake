package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class DriveBase extends Subsystem {

    private static DriveBase instance_ = new DriveBase();

    public static DriveBase getInstance() {
        return instance_;
    }

    private final TalonSRX leftMaster_, leftSlave_, rightMaster_, rightSlave_;
    private final ADXRS450_Gyro gyro_;

    // The constructor instantiates all of the drivetrain components when the
    // robot powers up

    private DriveBase() {
        leftMaster_ = new TalonSRX(Constants.id_driveLeftMaster);
        leftSlave_ = new TalonSRX(Constants.id_driveLeftSlave);
        rightMaster_ = new TalonSRX(Constants.id_driveRightMaster);
        rightSlave_ = new TalonSRX(Constants.id_driveRightSlave);
        gyro_ = new ADXRS450_Gyro(Constants.id_gyroSPIPort);

        // Start in open loop mode
        leftMaster_.set(ControlMode.PercentOutput, 0);
        leftSlave_.follow(leftMaster_);
    }

    public ADXRS450_Gyro getGyro() {
        return gyro_;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("gyro_angle", getGyro().getAngle());
        SmartDashboard.putBoolean("gyro_alive", getGyro().isConnected());
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {
        gyro_.reset();
    }
}
