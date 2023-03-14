// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class TwoCube extends SequentialCommandGroup {
  

  public TwoCube() {
    addCommands(
      RobotContainer.m_Arm.autoNudge(),
      RobotContainer.m_Arm.waitForArm(),
      RobotContainer.m_Arm.setPosition(1)
      // new ParallelCommandGroup(new DriveDist(), RobotContainer.m_Arm.flip()),
      // new ParallelCommandGroup(new DriveDistBack(), RobotContainer.m_Arm.flip()),
      // RobotContainer.m_Vision.setToBackPipeline(),     
      // new TurnToTarget(),
      // RobotContainer.m_Arm.setPosition(3)
    );
  }
}
