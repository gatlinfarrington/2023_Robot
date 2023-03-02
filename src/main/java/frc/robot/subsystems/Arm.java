// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import src.main.java.frc.robot.Constants;

public class Arm extends SubsystemBase {
  public enum ArmPosition{
    STARTING,
    FRONT_TOP,
    FRONT_MIDDLE,
    FRONT_BOTTOM,
    BACK_TOP,
    BACK_MIDDLE,
    BACK_BOTTOM
  }
  
  ArmPosition currentPosition = ArmPosition.STARTING;



  /** Creates a new ExampleSubsystem. */
  public Arm() {
    
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
  public void update () {
    switch(currentPosition) {
      case STARTING: 
        setArmPosition(Constants.STARTING_COUNT);
        break;
      case FRONT_TOP:
        setArmPosition(Constants.FRONT_TOP_COUNT);
        break;
      case FRONT_MIDDLE:
        setArmPosition(Constants.FRONT_MIDDLE_COUNT);
        break;
      case FRONT_BOTTOM:
        setArmPosition(Constants.FRONT_BOTTOM_COUNT);
        break;
      case BACK_TOP:
        setArmPosition(Constants.BACK_TOP_COUNT);
        break;
      case BACK_MIDDLE:
        setArmPosition(Constants.BACK_MIDDLE_COUNT);
        break;
      case BACK_BOTTOM:
        setArmPosition(Constants.BACK_BOTTOM_COUNT);
        break;


    }
  }
  public void setArmPosition(double position) {
    //DO NOTHING
  }
}
