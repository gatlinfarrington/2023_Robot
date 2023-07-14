// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveDist extends CommandBase {
  double inches;
  double speed;
  boolean done;
  public PIDController pid;

  /** Creates a new TurnAngle. */
  public DriveDist(double inches, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
    inches = -1*this.inches*2048*6/8/1.8; //constant found from encodercount*wheel Diamater*gear ratio * inches
    speed = this.speed;
    done = false;
    pid = new PIDController(0.01, 0, 0); //TUNE 
  }

  // Called when the command is i%itially scheduled.
  @Override
  public void initialize() {
    pid.enableContinuousInput(-180.0f,  180.0f); //TUNE
    pid.setTolerance(0, 0.1); //TUNE
    pid.setSetpoint(inches);
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_Drivetrain.getLeftEncoder() >= inches
    && RobotContainer.m_Drivetrain.getRightEncoder() >= inches) {
      done = true;
    }
    RobotContainer.m_Drivetrain.DriveTank(speed, speed);
    // double left_command = pid.calculate(RobotContainer.m_Drivetrain.getLeftEncoder());
    // double right_command = pid.calculate(RobotContainer.m_Drivetrain.getRightEncoder());
    // RobotContainer.m_Drivetrain.DriveTank(left_command, right_command);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
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