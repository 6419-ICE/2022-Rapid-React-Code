package frc.robot;

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
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.*;

public class TrajectoryPaths {

    private DriveTrain m_drive;

    public TrajectoryPaths(DriveTrain drive){
        m_drive = drive;
    }

    /** Creates a new TrajectoryAttempt. */
    public static Trajectory getTrajectoryAttempt() {
        DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.TrajectoryConstants.ksVolts,
                                        Constants.TrajectoryConstants.kvVoltSecondsPerMeter,
                                        Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
                                        Constants.TrajectoryConstants.m_driveKinematics,
                                        10);

        TrajectoryConfig config =
        new TrajectoryConfig(Constants.TrajectoryConstants.kMaxSpeedMetersPerSecond,
                                Constants.TrajectoryConstants.kMaxAccelerationMetersPerSecond)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.TrajectoryConstants.m_driveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
                ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3 , 0, new Rotation2d(0)),
            // Pass config
            config
        );

        return trajectory;
    }

    /** Creates a new TrajectoryAttempt. */
    public static Trajectory getTrajectoryTwoBalls() {
        DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.TrajectoryConstants.ksVolts,
                                        Constants.TrajectoryConstants.kvVoltSecondsPerMeter,
                                        Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
                                        Constants.TrajectoryConstants.m_driveKinematics,
                                        10);

        TrajectoryConfig config =
        new TrajectoryConfig(Constants.TrajectoryConstants.kMaxSpeedMetersPerSecond,
                                Constants.TrajectoryConstants.kMaxAccelerationMetersPerSecond)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.TrajectoryConstants.m_driveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(.5, 0)
                ),
            // End position
            new Pose2d(1.1, 0, new Rotation2d(-Math.toRadians(0))),
            // Pass config
            config
        );

        return trajectory;
    }

    public static Trajectory getPositionTrajectory(double x, double y) {



        DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.TrajectoryConstants.ksVolts,
                                        Constants.TrajectoryConstants.kvVoltSecondsPerMeter,
                                        Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
                                        Constants.TrajectoryConstants.m_driveKinematics,
                                        10);

        TrajectoryConfig config =
        new TrajectoryConfig(Constants.TrajectoryConstants.kMaxSpeedMetersPerSecond,
                                Constants.TrajectoryConstants.kMaxAccelerationMetersPerSecond)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.TrajectoryConstants.m_driveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(0, 0)
                ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3 , 0, new Rotation2d(Math.toRadians(x))),
            // Pass config
            config
        );

        return trajectory;
    }
}
