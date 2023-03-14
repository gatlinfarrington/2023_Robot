// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.DriveDist;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.TwoCube;
import frc.robot.commands.changePipeline;
import frc.robot.commands.flipLimelight;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

import java.security.spec.EncodedKeySpec;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final static Drivetrain m_Drivetrain = new Drivetrain();
  public final static Arm m_Arm = new Arm();
  public static Intake m_intake = new Intake();
  public static Vision m_Vision = new Vision();
  // Replace with CommandPS4Controller or CommandJoystick if needed
   public static  XboxController driverController = new XboxController(OperatorConstants.kDriverControllerPort);
   public static  XboxController coDriverController = new XboxController(OperatorConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings(
      // private final CommandXboxController Intake = 
    );
    m_Drivetrain.setDefaultCommand(new DriveArcade());
    m_intake.setDefaultCommand(new IntakeCommand());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    new JoystickButton(driverController, 1).onTrue(m_Arm.setPosition(3)); //b
    new JoystickButton(driverController, 2).onTrue(m_Arm.setPosition(1)); //b
    new JoystickButton(driverController, 4).onTrue(m_Arm.setPosition(2)); //y
    new JoystickButton(driverController, 3).onTrue(m_Arm.flip()); //x

    new JoystickButton(driverController, 5).onTrue(new TurnToTarget()); //left bumprt
    new JoystickButton(driverController, 6).onTrue(new DriveDist()); //right bumprt

    new JoystickButton(coDriverController, 1).onTrue(m_Drivetrain.invertDrive()); //co-drive a
    new JoystickButton(coDriverController, 2).onTrue(m_Arm.resetEncoder());
    
    new JoystickButton(coDriverController, 5).onTrue(new changePipeline()); //left bumper
    new JoystickButton(coDriverController, 6).onTrue(new flipLimelight()); //right bumper

    // new POVButton(driverController, ).onTrue(m_Arm.setPosition(1));
    // new Trigger(driverController.povUp(null)).onTrue(m_Arm.setPosition(2));
    // new Trigger(driverController.povLeft(null)).onTrue(m_Arm.flip());

    // driverController.povLeft(null).onTrue(m_Arm.setPosition(1));
    // driverController.povLeft(null).onTrue(m_Arm.setPosition(1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new TwoCube();
  }
}
