package frc.robot;

/*
* A locale for all robot constants.
* As of 01/13/2019, just a port of 2018 constants
 */

import edu.wpi.first.wpilibj.SPI;

public class Constants {

    /***********GYRO STUFF**********/

    //Gyro physical location
    public static SPI.Port id_gyroSPIPort = SPI.Port.kOnboardCS0;

    //TODO: Currently 10 sec (20190115) Needs empirical testing
    public static double kGyroCaliibrateAvgTime = 10;
    //TODO: Tune using the code in the Test Periodic (20190115)
    public static double kGyroTargetAngleThresh = 10;
    public static double kGyro_P = 0;

    /***********SPEED_CONTROLLER_IDs**********/

    //Drivetrain - Currently Using 2018 Standards
    public static final int id_driveLeftMaster = 1;
    public static final int id_driveLeftSlave = 0;
    public static final int id_driveRightMaster = 2;
    public static final int id_driveRightSlave = 3;

    //EVERYTHING below here is OLD (2018)
    // Todo: Update to actual 2019 values

    /***********AUTO***********/

    //Motion Profile
    public static final double kWheelBaseWidth = 1.0; //Measured in feet

    //Grabber2
    public static final double kIntakeCubeSpeed = -1;
    public static final double kEjectCubeSpeed = 1;
    public static final double kStopCubeSpeed = 0;

    /**********TELEOP**********/

    //PowerCube Manipulator Constants
    public static final float kEjectCubeSpeedMod = 1;
    public static final float kFoldArmsUp = 1;
    public static final float kFoldArmsDwn = -1;
    public static final double kJoyNeutralZone = 0.5;

    //Climber Constants
    public static final float kClimberUpSpeed = -1;

    //Drive Chassis and wheel and motor constants
    public static final double kChassisWheelDiameterInch = 6;
    public static final double kChassisWheelCircumferenceInch = kChassisWheelDiameterInch * Math.PI;

    //TalonSRX's now have multiple PID Loops so we want to select the first one
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 10;

    //Elevator PID parameters
    //TODO Tune these
    public static final double kElevateP = 1;
    public static final double kElevateI = 0;
    public static final double kElevateD = 45;

    //Drive Chassis PID parameters
    //TODO Tune these
    public static final double kDriveLeftP = 0;
    public static final double kDriveLeftI = 0;
    public static final double kDriveLeftD = 0;
    public static final double kDriveRightP = 0;
    public static final double kDriveRightI = 0;
    public static final double kDriveRightD = 0;
    public static final double kLinearClosedLoop_Tolerance_Default = 1024;

    public static final double kHeadingClosedLoop_P = 0;
    public static final double kHeadingClosedLoop_I = 0;
    public static final double kHeadingClosedLoop_D = 0;
    public static final double kHeadingClosedLoop_Tolerance_Default = 1;

    //Elevator

    //public static final int kElevatorDefaultElevateSpeed = 1; TODO Possibly unneeded
    public static final int kElevatorMaxEncPos = 4000;
    public static final int kElevatorHigh = 32000;
    public static final int kElevatorMedium = 16000;
    public static final int kElevatorLow = 3;
    public static final int kIntegralZone = 700;
    public static final int id_descend_limit = 2; //TODO: Find real input number
    public static final int kElevateAllowableError = 10;


    //Robot Ports - These should match up to TODO: create Google doc to outline motor controller ports



    //TODO: Set these to reality once the electronics is laid out. Sparks?
    //PowerCube Manipulator Speed Controllers
    public static final int id_grabber_wheels = 1;
    public static final int id_grabber_flipper_upper = 0;
    //PowerCube Grabber Limit Switch
    public static final int id_intake_limit = 1; //TODO: Find real input ID

    //Elevator Speed Controllers
    public static final int id_elevateMaster = 4;
    public static final int id_elevateSlave = 5;


    public static double kAutoUpdateRate = 1.0 / 50.0;


    // Motion Profile Params
    public static final double kTicksPerMeter = 2607.5945876176131812373915790953 * 1 / 0.3048; //* 2) / 7; Kinda works for turning = 3650;
    public static double kMetersPerFoot = 0.3048;
    public static final double kTicksPerFoot = 2607.5945876176131812373915790953; //* 2) / 7; Kinda works for turning = 3650;
    public static final double kSensorUnitsPerRot = 4096; //* 2) / 7;
    public static int kBaseTrajPeriodMs = 50;

}
