// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  ArmPosition lastPosition;
  //motor declarations
  TalonFX ArmMotor = new TalonFX(Constants.ARM_MOTOR_PORT);
  TalonFX ArmMotor_slave = new TalonFX(Constants.ARM_MOTOR_SLAVE_PORT);
  CANSparkMax ExtendMotor = new CANSparkMax(Constants.ARM_EXTEND_PORT, MotorType.kBrushless);


  /** Creates a new ExampleSubsystem. */
  public Arm() {
    ArmMotor_slave.follow(ArmMotor);
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
        lastPosition = ArmPosition.STARTING;
        break;
      case FRONT_TOP:
        if(lastPosition == ArmPosition.BACK_TOP){
          retractArm();
        }
        setArmPosition(Constants.FRONT_TOP_COUNT);
        extendArm();
        lastPosition = ArmPosition.FRONT_TOP;
        break;
      case FRONT_MIDDLE:
        if(lastPosition == ArmPosition.BACK_TOP || lastPosition == ArmPosition.FRONT_TOP){
          retractArm();
        }
        setArmPosition(Constants.FRONT_MIDDLE_COUNT);
        lastPosition = ArmPosition.FRONT_MIDDLE;
        break;
      case FRONT_BOTTOM:
        if(lastPosition == ArmPosition.BACK_TOP || lastPosition == ArmPosition.FRONT_TOP){
          retractArm();
        }
        setArmPosition(Constants.FRONT_BOTTOM_COUNT);
        lastPosition = ArmPosition.FRONT_BOTTOM;
        break;
      case BACK_TOP:
        if(lastPosition == ArmPosition.FRONT_TOP){
          retractArm();
        }
        setArmPosition(Constants.BACK_TOP_COUNT);
        extendArm();
        lastPosition = ArmPosition.BACK_TOP;
        break;
      case BACK_MIDDLE:
        if(lastPosition == ArmPosition.BACK_TOP || lastPosition == ArmPosition.FRONT_TOP){
          retractArm();
        }
        setArmPosition(Constants.BACK_MIDDLE_COUNT);
        lastPosition = ArmPosition.BACK_MIDDLE;
        break;
      case BACK_BOTTOM:
        if(lastPosition == ArmPosition.BACK_TOP || lastPosition == ArmPosition.FRONT_TOP){
          retractArm();
        }
        setArmPosition(Constants.BACK_BOTTOM_COUNT);
        lastPosition = ArmPosition.BACK_BOTTOM;
        break;


    }
  }
  public void setArmPosition(double position) {
    //DO NOTHING
    ArmMotor.set(TalonFXControlMode.Position, position);
  }
  public void extendArm(){
    //extend the arm
    // ExtendMotor.
  }
  public void retractArm(){
    //retract the arm
    //motor.set(position, controlMode.position)
  }
}
