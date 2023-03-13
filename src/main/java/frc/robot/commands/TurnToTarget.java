// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;

import org.ejml.dense.row.mult.MatrixVectorMult_DDRM;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToTarget extends CommandBase {
  boolean m_done;
  public TurnToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
    addRequirements(RobotContainer.m_Vision);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    // RobotContainer.m_Drivetrain.setBrakeMode();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_done = RobotContainer.m_Vision.targetTurn(RobotContainer.m_Drivetrain);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Drivetrain.arcadeDrive(0, 0);
    // RobotContainer.m_Drivetrain.setCoastMode();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return m_done;
  }
}