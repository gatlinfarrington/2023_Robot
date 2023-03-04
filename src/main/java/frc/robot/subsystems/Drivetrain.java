// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  WPI_TalonFX right1 = new WPI_TalonFX(motorPortConstants.DRIVE_TRAIN_RIGHT_1);
  WPI_TalonFX right2 = new WPI_TalonFX(motorPortConstants.DRIVE_TRAIN_RIGHT_2);

  WPI_TalonFX left1 = new WPI_TalonFX(motorPortConstants.DRIVE_TRAIN_LEFT_1);
  WPI_TalonFX left2 = new WPI_TalonFX(motorPortConstants.DRIVE_TRAIN_LEFT_2);


  MotorControllerGroup leftMotors = new MotorControllerGroup(left1, left2);
  MotorControllerGroup rightMotors = new MotorControllerGroup(right1, right2 );

  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  public Drivetrain() {
    
  }


  public void arcadeDrive(double moveSpeed, double rotateSpeed) {
    drive.arcadeDrive(moveSpeed, rotateSpeed);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
