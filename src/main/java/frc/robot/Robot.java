// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final Joystick m_stick = new Joystick(0);
  private final TalonSRX arm = new TalonSRX(0);
  private final TalonSRX left = new TalonSRX(1);
  private final TalonSRX right = new TalonSRX(2);
  private final DifferentialDrive m_robot = new DifferentialDrive(left, right);
  private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);
  private final PIDController turnController;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    right.configFactoryDefault();
    left.configFactoryDefault();
    arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    arm.setSelectedSensorPosition(0);
    arm.configNeutralDeadband(0.001);
    arm.selectProfileSlot(0, 0);
    arm.config_kP(0, 0.05);
    arm.config_kI(0, 0.0);
    arm.config_kD(0, 0.0);
    arm.configMotionCruiseVelocity(800);
    arm.configMotionAcceleration(200);

    gyro.enableLogging(true);
    gyro.calibrate();
    gyro.reset();
    turnController = new PIDController(0.05, 0.0, 0.0);
    turnController.enableContinuousInput(-180f, 180f);
    turnController.setIntegratorRange(-0.5, 0.5);
    turnController.setTolerance(2); 
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /* show gyro values, a button to go straight and 4 buttons to turn to the target angle */
    /* to do */
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    /* show Encoder values, a button to move the arm to the right angle */
    /* to do */
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
