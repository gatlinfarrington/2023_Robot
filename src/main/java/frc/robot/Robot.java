// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.EncoderConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }
 

//Use for Intake Motors Spark Max ID 5 and 6
  // CANSparkMax testArm = new CANSparkMax(5, MotorType.kBrushless);
  // CANSparkMax testArm2 = new CANSparkMax(6, MotorType.kBrushless);
  //Use for Arm 
  // TalonFX testArm = new TalonFX(10);
  // Servo s = new Servo(1);
  // XboxController xbController = new XboxController(0);
  // //Speed for Anything
  // public double testSpeed = 1;
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // testArm.setSelectedSensorPosition(0);
  }

  TalonFX Arm = new TalonFX(10);
  // DigitalInput lSwitch = new DigitalInput(0);
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //Use for CANSparkMax (Intake Motors)
    // testArm.set(-testSpeed);//OUTTAKING
    // testArm2.set(testSpeed);
    //Use for TalonFX motor (ARM)
    //testArm.set(ControlMode.PercentOutput, testSpeed);
    // if(xbController.getRawButtonPressed(1)){
    //   testArm.setSelectedSensorPosition(0);
    //   s.setAngle(0);
    // }else if(xbController.getRawButtonPressed(2)){
    //   s.setAngle(180);
    // }
    // System.out.println(testArm.getSelectedSensorPosition());
      // System.out.println("Switch: " + lSwitch.get());
    if(RobotContainer.driverController.getRightBumper()){
      Arm.set(ControlMode.Position, EncoderConstants.FRONT_MIDDLE_COUNT);
    }else if(RobotContainer.driverController.getLeftBumper()){
      Arm.set(ControlMode.Position, EncoderConstants.BACK_MIDDLE_COUNT);
    }else{
      Arm.set(ControlMode.PercentOutput, 0);
    }

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
