// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveDist;
import frc.robot.commands.DriveDistBack;
import frc.robot.commands.TurnAngle;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.ArmCommands.flipArmParallel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeCube extends SequentialCommandGroup {
  /** Creates a new ThreeCube. */
  public ThreeCube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.m_Drivetrain.resetGyro(),
      RobotContainer.m_Arm.autoNudge(),
      RobotContainer.m_Arm.waitForArmThreeCube(),
      RobotContainer.m_Arm.setPosition(2),
      new ParallelCommandGroup(new DriveDist(135, Constants.DRIVE_SPEED), new flipArmParallel()), //TUNE
      new ParallelCommandGroup(new DriveDistBack(165), new flipArmParallel()), //TUNE
      RobotContainer.m_Arm.setPosition(2),
      RobotContainer.m_Vision.setToBackPipeline(),     
      new TurnToTarget(),
      new ParallelCommandGroup(new TurnAngle(45), new flipArmParallel()), //TUNE
      new DriveDist(50, Constants.DRIVE_SPEED), //TUNE THIS
      new ParallelCommandGroup(new DriveDistBack(50), new flipArmParallel()), //TUNE
      new TurnAngle(0), //TUNE
      new DriveDistBack(125), //TUNE
      RobotContainer.m_Arm.setPosition(3)

    );
  }
}
