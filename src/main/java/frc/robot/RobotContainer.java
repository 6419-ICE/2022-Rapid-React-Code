// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Intake m_intake = new Intake();

  private final HandleDriveTrain m_handleDriveTrain = new HandleDriveTrain(m_driveTrain);
  private final HandleIntake m_handleIntake = new HandleIntake(m_intake);

  private static Joystick leftJoystick;
  private static Joystick rightJoystick;
  private static Joystick mechanismJoystick;
  private static Joystick gamepadController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_driveTrain.setDefaultCommand(m_handleDriveTrain);
    m_intake.setDefaultCommand(m_handleIntake);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*leftJoystick = new Joystick(Constants.joy1);
    rightJoystick = new Joystick(Constants.joy2);
    mechanismJoystick = new Joystick(Constants.joy3);*/
    gamepadController = new Joystick(Constants.gamepadJoy);

  }
  
  /*
  public static Joystick getLeftJoy() {
    return leftJoystick;
  }

  /** Return the right Joystick 
  public static Joystick getRightJoy() {
    return rightJoystick;
  }

  public static Joystick getMechanismJoystick() {
    return mechanismJoystick;
  }*/

  public static Joystick getGamepad() {
    return gamepadController;
  }

  public static boolean getLowerIntakeButton() {
    return gamepadController.getRawButton(Constants.kGamepadButtonShoulderL);
  }

  public static boolean getRaiseIntakeButton() {
    return gamepadController.getRawButton(Constants.kGamepadButtonShoulderR);
  }

  public static boolean getRunIntakeButton() {
    return gamepadController.getRawAxis(2) > .7;
  }

  public static boolean getReverseIntakeButton() {
    return gamepadController.getRawAxis(3) > .7;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_handleDriveTrain;
  }
}
