// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveDist extends CommandBase {
  double in;
  public PIDController pid;

  /** Creates a new TurnAngle. */
  public DriveDist(double inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
    in = -1*inches*2048*6/8/1.8; //constant found from encodercount*wheel Diamater*gear ratio * inches
    pid = new PIDController(0, 0, 0); //TUNE 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.enableContinuousInput(-180.0f,  180.0f); //TUNE
    pid.setTolerance(0, 0.1); //TUNE
    pid.setSetpoint(in);
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left_command = pid.calculate(RobotContainer.m_Drivetrain.getRobotYaw());
    double right_command = pid.calculate(RobotContainer.m_Drivetrain.getRobotYaw());
    RobotContainer.m_Drivetrain.DriveTank(left_command, right_command*-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
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