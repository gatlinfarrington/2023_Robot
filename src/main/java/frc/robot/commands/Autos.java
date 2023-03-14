// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }
  public static CommandBase twoCube(Drivetrain d, Intake i, Arm a, Vision v){
    return Commands.sequence(
      a.autoNudge(),
      
      a.setPosition(2),
      flipAndRotate(a)
      
      
    );
  }

  public static CommandBase flipAndRotate(Arm a){
    return Commands.parallel(new DriveDist(), a.flip());
  }

  public static CommandBase flipAndRotateBack(Arm a){
    return Commands.parallel(new DriveDist(), a.flip());
  }

  // public static CommandBase waitForArm(){
    
  // }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
