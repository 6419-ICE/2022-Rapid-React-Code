// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TrajectoryAttempt extends CommandBase {

  private DriveTrain m_drive;
  private Trajectory trajectory;
  private TrajectoryConfig config;
  private RamseteCommand ramseteCommand;

  /** Creates a new TrajectoryAttempt. */
  public TrajectoryAttempt(DriveTrain drivetrain) {
    m_drive = drivetrain;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetHeading();
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.TrajectoryConstants.ksVolts,
                                   Constants.TrajectoryConstants.kvVoltSecondsPerMeter,
                                   Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
                                   Constants.TrajectoryConstants.m_driveKinematics,
                                   10);

    config =
      new TrajectoryConfig(Constants.TrajectoryConstants.kMaxSpeedMetersPerSecond,
                            Constants.TrajectoryConstants.kMaxAccelerationMetersPerSecond)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(Constants.TrajectoryConstants.m_driveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);
    
    trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
          new Translation2d(1, 0)
          ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    m_drive.resetOdometry(trajectory.getInitialPose());

     ramseteCommand = new RamseteCommand(
      trajectory,
      m_drive::getPose,
      new RamseteController(Constants.TrajectoryConstants.kRamseteB, Constants.TrajectoryConstants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.TrajectoryConstants.ksVolts,
                                 Constants.TrajectoryConstants.kvVoltSecondsPerMeter,
                                 Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
      Constants.TrajectoryConstants.m_driveKinematics,
      m_drive::getWheelSpeeds,
      new PIDController(Constants.TrajectoryConstants.kpDriveVel, 0, 0),
      new PIDController(Constants.TrajectoryConstants.kpDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_drive::tankDriveVolts,
      m_drive
  );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ramseteCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
