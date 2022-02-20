// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.armStates;
import frc.robot.subsystems.Shooter.shooterStates;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriveTrain m_driveTrain = new DriveTrain();
  public static final Intake m_intake = new Intake();
  public static final Uptake m_uptake = new Uptake();
  public static final Shooter m_shooter = new Shooter();

  private final HandleDriveTrain m_handleDriveTrain = new HandleDriveTrain(m_driveTrain);
  private final HandleIntake m_handleIntake = new HandleIntake(m_intake, m_uptake);
  private final HandleUptakeShooter m_handleShooter = new HandleUptakeShooter(m_uptake, m_shooter, shooterStates.LOW);
  private final HandleUptakeShooter m_shootLow = new HandleUptakeShooter(m_uptake, m_shooter, shooterStates.LOW);
  private final HandleUptakeShooter m_shootHigh = new HandleUptakeShooter(m_uptake, m_shooter, shooterStates.HIGH);

  private static Joystick leftJoystick;
  private static Joystick rightJoystick;
  private static Joystick mechanismJoystick;
  private static Joystick gamepadController;

  private static SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_driveTrain.setDefaultCommand(m_handleDriveTrain);
    m_intake.setDefaultCommand(m_handleIntake);
    m_uptake.setDefaultCommand(m_handleShooter);
    m_shooter.setDefaultCommand(m_handleShooter);

    // Configure Autonomous Selections
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Shoot Low", new ShootLowBackAwayAuto(m_shooter, m_uptake));
    autoChooser.addOption("Shoot High", new ShootHighBackAwayAuto());

    configureButtonBindings();

    SmartDashboard.putData("Autonomous", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*leftJoystick = new Joystick(Constants.joy1);
    rightJoystick = new Joystick(Constants.joy2);*/
    mechanismJoystick = new Joystick(Constants.buttonBox);
    gamepadController = new Joystick(Constants.gamepadJoy);
    /*
    JoystickButton shootLowButton = new JoystickButton(gamepadController, 2);
    JoystickButton shootHighButton = new JoystickButton(gamepadController, 3);

    shootLowButton.whenHeld(new HandleUptakeShooter(m_uptake, m_shooter, shooterStates.LOW));
    shootHighButton.whenHeld(m_shootHigh);*/

  }
  
  /*
  public static Joystick getLeftJoy() {
    return leftJoystick;
  }

  /** Return the right Joystick 
  public static Joystick getRightJoy() {
    return rightJoystick;
  }*/

  public static Joystick getMechanismJoystick() {
    return mechanismJoystick;
  }

  public static Joystick getGamepad() {
    return gamepadController;
  }

  // public static boolean getLowerIntakeButton() {
  //   return -mechanismJoystick.getRawAxis(Constants.gamepadConstants.moveIntakeAxis) < 0;
  // }

  // public static boolean getRaiseIntakeButton() {
  //   return -mechanismJoystick.getRawAxis(Constants.gamepadConstants.moveIntakeAxis) > 0;
  // }

  public static boolean getRunIntakeButton() {
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.runIntakeButton);
  }

  public static boolean getReverseIntakeButton() {
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.reverseIntakeButton);
  }

  public static boolean getRunUptakeButton(){
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.runUptakeButton);
  }

  public static boolean getReverseUptakeButton() {
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.reverseUptakeButton);
  }

  public static boolean getShooterHighButton(){
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.shooterHighButton);
  }

  public static boolean getShooterLowButton(){
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.shooterLowButton);
  }

  public static double getDriveTrainForward(){
    return Utilities.applyDeadband(-gamepadController.getRawAxis(Constants.gamepadConstants.kGamepadAxisLeftStickY), 0.03);
  }

  public static double getDriveTrainTurn(){
    return Utilities.applyDeadband(gamepadController.getRawAxis(Constants.gamepadConstants.kGamepadAxisRightStickX), 0.03);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}