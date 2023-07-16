// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveDist extends CommandBase {
  double inches;
  double speed;
  boolean done;
  double angle;
  public PIDController pid;

  /** Creates a new TurnAngle. */
  public DriveDist(double ang, double spd, double kp, double ki, double kd, double inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
    RobotContainer.m_Drivetrain.resetLeftEncoder();
    RobotContainer.m_Drivetrain.resetRightEncoder();
    RobotContainer.m_Drivetrain.resetGyro();
    inches = this.inches*2048*8/7.5; //constant found from encodercount*wheel Diamater*gear ratio * inches
    speed = spd;
    done = false;
    angle = ang;
    pid = new PIDController(kp, ki, kd); //TUNE 
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Drivetrain.resetGyro();
    pid.enableContinuousInput(-180.0f,  180.0f); //TUNE
    pid.setTolerance(0, 0.1); //TUNE
    pid.setSetpoint(angle);
    pid.reset();
    RobotContainer.m_Drivetrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left_command = -1 * pid.calculate(RobotContainer.m_Drivetrain.getRobotYaw());
    double right_command = pid.calculate(RobotContainer.m_Drivetrain.getRobotYaw());
    System.out.println("Left: " + left_command);
    System.out.println("Right: " + right_command);
    System.out.println("Error: " + pid.getPositionError());

    RobotContainer.m_Drivetrain.DriveTank((speed + left_command), (speed + right_command) * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((RobotContainer.m_Drivetrain.getLeftEncoder() - RobotContainer.m_Drivetrain.getRightEncoder())/2 > inches);
  }
}


// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.ExampleSubsystem;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class DriveDist extends CommandBase {
//   boolean done;
//   int in;

//   public DriveDist(int inches) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(RobotContainer.m_Drivetrain);
//     in = inches;
//   }

//   // Called just before this Command runs the first time
//   @Override
//   public void initialize() {
    
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   public void execute() {
//     done = RobotContainer.m_Drivetrain.driveDistance(in);
//     // done = RobotContainer.m_Drivetrain.driveDistance(155);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     RobotContainer.m_Drivetrain.arcadeDrive(0, 0);
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   public boolean isFinished() {
//     return done;
//   }
// }