// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TrajectoryCommand extends RamseteCommand {
  private DriveTrain m_drive;
  private Trajectory trajectory;
  
  private static PIDController leftController = new PIDController(Constants.TrajectoryConstants.kpDriveVel, 0, Constants.TrajectoryConstants.kdDriveVel);
  private static PIDController rightController = new PIDController(Constants.TrajectoryConstants.kpDriveVel, 0, Constants.TrajectoryConstants.kdDriveVel);
  /** Creates a new TrajectoryCommand. */
  public TrajectoryCommand(
    Trajectory trajectory,
    DriveTrain drive) {
      super(
      trajectory,
      drive::getPose,
      new RamseteController(Constants.TrajectoryConstants.kRamseteB, Constants.TrajectoryConstants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.TrajectoryConstants.ksVolts,
                                  Constants.TrajectoryConstants.kvVoltSecondsPerMeter,
                                  Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
      Constants.TrajectoryConstants.m_driveKinematics,
      drive::getWheelSpeeds,
      new PIDController(Constants.TrajectoryConstants.kpDriveVel, 0, Constants.TrajectoryConstants.kdDriveVel),
      new PIDController(Constants.TrajectoryConstants.kpDriveVel, 0, Constants.TrajectoryConstants.kdDriveVel),
      drive::tankDriveVolts,
      drive
      );
    m_drive = drive;
    this.trajectory = trajectory;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setMaxMotorSpeed(Constants.DrivetrainConstants.speedLmt);
    m_drive.resetHeading();
    m_drive.resetOdometry(trajectory.getInitialPose());
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("execute");
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
