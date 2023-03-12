// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;

public class Arm extends SubsystemBase {
  public enum ArmPosition{
    STARTING,
    FRONT,
    BACK
  }
  
  ArmPosition currentPosition = ArmPosition.STARTING;
  ArmPosition lastPosition;

  //motor declarations
  TalonFX ArmMotor = new TalonFX(motorPortConstants.ARM_MOTOR_PORT);

  Servo limelightServo = new Servo(ServoPortConstants.LIMELIGHT_SERVO_PORT);
  public boolean isLimeLightFront = true;

  /** Creates a new ExampleSubsystem. */
  public Arm() {
    // ArmMotor_slave.follow(ArmMotor);
    ArmMotor.setNeutralMode(NeutralMode.Brake);
    ArmMotor.setSelectedSensorPosition(0);
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
  public CommandBase setPosition(int position){
    if(position == 1){
        return runOnce(() -> {
                if(ArmMotor.getSelectedSensorPosition() > 0){
                    while(ArmMotor.getSelectedSensorPosition() > EncoderConstants.FRONT_MIDDLE_COUNT - 100){
                        ArmMotor.set(ControlMode.Position, -.3);
                    }
                    ArmMotor.set(ControlMode.PercentOutput, 0);
                    RobotContainer.m_intake.dispense(.5, 1);
                }else{
                    while(ArmMotor.getSelectedSensorPosition() < EncoderConstants.BACK_MIDDLE_COUNT + 100){
                        ArmMotor.set(ControlMode.PercentOutput, .3);
                    }
                    ArmMotor.set(ControlMode.PercentOutput, 0);
                    RobotContainer.m_intake.dispense(.5, 1);
                }
            }
        );
    }else if(position == 2){
        return runOnce(() -> {
            if(ArmMotor.getSelectedSensorPosition() > 0){
                    while(ArmMotor.getSelectedSensorPosition() > EncoderConstants.FRONT_TOP_COUNT - 100){
                        ArmMotor.set(ControlMode.Position, -.3);
                    }
                    ArmMotor.set(ControlMode.PercentOutput, 0);
                    RobotContainer.m_intake.dispense(.8, 1);
                }else{
                    while(ArmMotor.getSelectedSensorPosition() < EncoderConstants.BACK_TOP_COUNT + 100){
                        ArmMotor.set(ControlMode.PercentOutput, .3);
                    }
                    ArmMotor.set(ControlMode.PercentOutput, 0);
                    RobotContainer.m_intake.dispense(.8, 1);
                }
            }
        );
    }else{
        return runOnce(() -> {
            if(ArmMotor.getSelectedSensorPosition() > 0){
                    while(ArmMotor.getSelectedSensorPosition() < position + 100){
                        ArmMotor.set(ControlMode.Position, .3);
                    }
                    ArmMotor.set(ControlMode.PercentOutput, 0);
                    RobotContainer.m_intake.dispense(.8, 1);
                }else{
                    while(ArmMotor.getSelectedSensorPosition() > position - 100){
                        ArmMotor.set(ControlMode.PercentOutput, -.3);
                    }
                    ArmMotor.set(ControlMode.PercentOutput, 0);
                    RobotContainer.m_intake.dispense(.8, 1);
                }
            }
        );
    }
        
    
  }

  public CommandBase flip(){
    return runOnce(
        () ->{
            if(ArmMotor.getSelectedSensorPosition() > 0){
                while(ArmMotor.getSelectedSensorPosition() > -1000){
                    ArmMotor.set(ControlMode.Position, -.25);
                }
                ArmMotor.set(ControlMode.PercentOutput, 0);
            }else{
                while(ArmMotor.getSelectedSensorPosition() <  1000){
                    ArmMotor.set(ControlMode.PercentOutput, .25);
                }
                ArmMotor.set(ControlMode.PercentOutput, 0);
            }
            flipLimeServo();
        }
    );
    
  }


  public void flipLimeServo(){
    if(isLimeLightFront){
        limelightServo.setAngle(180);
    }else{
        limelightServo.setAngle(0);
    }
    isLimeLightFront = !isLimeLightFront;
  }

  public void setArmPosition(double position) {
    //DO NOTHING
    ArmMotor.set(TalonFXControlMode.Position, position);
  }

  public int getDirection(){
    if(currentPosition == ArmPosition.STARTING){
        return 99;
    }else if(currentPosition == ArmPosition.FRONT){
        return 1;
    }else{
        return -1;
    }
  }

  public double getEncoderPosition(){
    return ArmMotor.getSelectedSensorPosition();
  }

  public void resetEncoder(){
    ArmMotor.setSelectedSensorPosition(0);
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
