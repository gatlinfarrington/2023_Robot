// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private PIDController pidController;
    double kP = 0.1;
    double kI = 0.0;
    double kD = 0.0;
    double kF = 0.0;
    int iZone = 0;
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // testArm.setSelectedSensorPosition(0);
    pidController = new PIDController(0.1, 0, 0);
    Arm.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    Arm.setSelectedSensorPosition(0);

        // Set up the PID controller
    
    Arm.config_kP(0, kP, 0);
    Arm.config_kI(0, kI, 0);
    Arm.config_kD(0, kD, 0);
    Arm.config_kF(0, kF, 0);
    Arm.config_IntegralZone(0, iZone, 0);
  }

  TalonFX Arm = new TalonFX(10);
  XboxController tController = new XboxController(1);
  // DigitalInput lSwitch = new DigitalInput(0);
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
    
    double setpoint = EncoderConstants.FRONT_MIDDLE_COUNT;
    // double position = Arm.getSelectedSensorPosition();
    // double output = pidController.calculate(position, setpoint);
    if(tController.getLeftTriggerAxis() > 0.6){
      double position = Arm.getSelectedSensorPosition(0);

      // Calculate the error and output of the PID controller
      double error = setpoint - position;
      double output = kP * error + kI * Arm.getIntegralAccumulator() + kD * Arm.getActiveTrajectoryVelocity() + kF * setpoint;

      // Set the output of the TalonFX motor controller
      Arm.set(ControlMode.PercentOutput, output);
      System.out.println("MOVING ARM");
    }else{
      Arm.set(ControlMode.PercentOutput, 0);
    }
    // if(tController.getRightBumper()){
    //   Arm.set(ControlMode.PercentOutput, 0);
    // }
    // System.out.println("Output: " + output + " \n Current Encoder Count: " + Arm.getSelectedSensorPosition());
      // Get the current position
      
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
