// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.armStates;
import frc.robot.subsystems.Shooter.shooterStates;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.gamepadConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final DriveTrain m_driveTrain = new DriveTrain();
  private static final Intake m_intake = new Intake();
  private static final Uptake m_uptake = new Uptake();
  private static final Shooter m_shooter = new Shooter();
  private static final Hanger m_hanger = new Hanger();
  public static final Limelight m_limelight = new Limelight();

  private TrajectoryPaths m_paths = new TrajectoryPaths(m_driveTrain);

  private final HandleDriveTrain m_handleDriveTrain = new HandleDriveTrain(m_driveTrain);
  private final HandleIntake m_handleIntake = new HandleIntake(m_intake);
  private final HandleUptake m_handleUptake = new HandleUptake(m_uptake);
  private final HandleShooter m_handleShooter = new HandleShooter(m_shooter);
  private final HandleHanger m_handleHanger = new HandleHanger(m_hanger);
  private final HandleLimelight m_handleLimelight = new HandleLimelight(m_limelight);
  private final AutonomousCenterOnGoal m_centerOnGoal = new AutonomousCenterOnGoal(m_driveTrain, m_limelight);
  private final DriveByEncoder m_driveByEncoder = new DriveByEncoder(m_driveTrain, 36);
  private final PIDTurn m_PIDTurn = new PIDTurn(m_driveTrain, 20);
  private final TrajectoryAttempt m_trajectoryAttempt = new TrajectoryAttempt(m_driveTrain);
  private final TrajectoryCommand m_trajectoryAttempt2 =  new TrajectoryCommand(TrajectoryPaths.getTrajectoryAttempt(), m_driveTrain); 
  private static Joystick mechanismJoystick;
  private static Joystick gamepadController;

  
  

  private static SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings

    configureButtonBindings();
    m_driveTrain.setDefaultCommand(m_handleDriveTrain);
    m_intake.setDefaultCommand(m_handleIntake);
    m_uptake.setDefaultCommand(m_handleUptake);
    m_shooter.setDefaultCommand(m_handleShooter);
    m_hanger.setDefaultCommand(m_handleHanger);
    m_limelight.setDefaultCommand(m_handleLimelight);

    // Configure Autonomous Selections
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Shoot Low", new ShootLowBackAwayAuto(m_shooter, m_uptake, m_driveTrain, m_intake));
    autoChooser.addOption("Shoot High", new ShootHighBackAwayAuto(m_shooter, m_uptake, m_driveTrain));
    autoChooser.addOption("Drive By Encoder", m_driveByEncoder);
    autoChooser.addOption("PID Turn", m_PIDTurn);
    autoChooser.addOption("Center on Goal", new AutonomousCenterOnGoal(m_driveTrain, m_limelight));
    autoChooser.addOption("Only Shoot", new AutonomousShoot(m_uptake, m_shooter, shooterStates.HIGH, 10000));
    autoChooser.addOption("Trajectory Attempt 2", new TrajectoryCommand(TrajectoryPaths.getTrajectoryAttempt(), m_driveTrain));
    autoChooser.addOption("Trajectory Attempt", m_trajectoryAttempt);
    autoChooser.addOption("Two Ball Auto", new TwoBallAuto(m_driveTrain, m_intake, m_uptake, m_shooter, m_limelight));

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


    JoystickButton shootLowButton = new JoystickButton(mechanismJoystick, Constants.gamepadConstants.shooterLowButton);
    JoystickButton shootHighButton = new JoystickButton(mechanismJoystick, Constants.gamepadConstants.shooterHighButton);
    JoystickButton spoolUpLowButton = new JoystickButton(mechanismJoystick, Constants.gamepadConstants.spoolUpLowButton);
    JoystickButton spoolUpHighButton = new JoystickButton(mechanismJoystick, Constants.gamepadConstants.spoolUpHighButton);
    JoystickButton centerOnGoalButton = new JoystickButton(mechanismJoystick, Constants.gamepadConstants.centerButton);

    shootLowButton.whenHeld(new TurretShoot(m_uptake, m_shooter, shooterStates.LOW), false);
    shootHighButton.whenHeld(new TurretShoot(m_uptake, m_shooter, shooterStates.HIGH), false);
    spoolUpLowButton.whenHeld(new TurretSpool(m_uptake, m_shooter, shooterStates.LOW), false);
    spoolUpHighButton.whenHeld(new TurretSpool(m_uptake, m_shooter, shooterStates.HIGH), false);
    centerOnGoalButton.whenPressed(new AutonomousCenterOnGoal(m_driveTrain, m_limelight), false);
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

  public static boolean getLowerIntakeButton() {
    return -mechanismJoystick.getRawAxis(Constants.gamepadConstants.moveIntakeAxis) < -.5;
  }
 
  public static boolean getRaiseIntakeButton() {
    return -mechanismJoystick.getRawAxis(Constants.gamepadConstants.moveIntakeAxis) > .5;
  } 
  
  public static boolean getLowerHangerButton() {
    return -mechanismJoystick.getRawAxis(Constants.gamepadConstants.moveHangerAxis) < -.5;
  }
 
  public static boolean getRaiseHangerButton() {
    return -mechanismJoystick.getRawAxis(Constants.gamepadConstants.moveHangerAxis)  > .5;
  }

  public static boolean getRunIntakeButton() {
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.runIntakeButton);
  }

  public static boolean getReverseIntakeButton() {
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.reverseIntakeButton);
  }

  public static boolean getRunUptakeButton(){
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.runUptakeButton);
  }

  public static boolean getShooterButton() {
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.spoolUpFire);
  }

  public static boolean getShooterHighButton(){
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.shooterHighButton);
  }

  public static boolean getShooterLowButton(){
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.shooterLowButton);
  }

  public static boolean getCenterButton(){
    return mechanismJoystick.getRawButton(Constants.gamepadConstants.centerButton);
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
    return  autoChooser.getSelected();
  }
}