/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveBase;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Controllers mControls = Controllers.getInstance();
  DriveBase mDrive = DriveBase.getInstance();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //Restart Robot Code @ beginning of every match. This will zero gyro. Takes ~10 sec
    //mDrive.calibrateGyro();
    // mDrive.getGyro().calibrate(); //Switched because it was delaying robot init
    mDrive.zeroSensors();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    mDrive.outputToSmartDashboard();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
//TODO: Needs testing
    mDrive.getDrive().arcadeDrive(mControls.getThrottle(), mControls.getTurn());
  }

  public void disabledPeriodic(){
    mDrive.stop();
  }

  public void testInit(){
      //TODO: Why doesn't smartdash seem to work here? Maybe try shuffleboard
    SmartDashboard.putBoolean("enableGyroSpinTest", false);
    SmartDashboard.putNumber("spin_to", 0);
    SmartDashboard.putNumber("GyroPGain:", 0);

    mDrive.zeroSensors();
    mDrive.calibrateGyro();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
//     boolean enableGyroSpinTesting = SmartDashboard.getBoolean("enableGyroSpinTest", false);
//     double kP = SmartDashboard.getNumber("GyroPGain:", 0);
//     if (enableGyroSpinTesting){
//         double angle = SmartDashboard.getNumber("Spin Gyro to:", 10);
//         double turningValue = (angle - mDrive.getGyro().getAngle()) * kP;
//         // Invert the direction of the turn if we are going backwards
//         turningValue = Math.copySign(turningValue, m_joystick.getY());
//         mDrive.getDrive().arcadeDrive(m_joystick.getY(), turningValue);
//     } else {
//         mDrive.getDrive().arcadeDrive(m_joystick.getY(), m_joystick.getX());
//     }
    mDrive.turnToAngle(120);
    mDrive.outputToSmartDashboard();

  }
}
