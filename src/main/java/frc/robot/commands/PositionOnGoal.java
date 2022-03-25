// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shooterStates;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PositionOnGoal extends SequentialCommandGroup {
  /** Creates a new PositionOnGoal. */

  /** Creates a new PositionOnGoal. */
  public PositionOnGoal(DriveTrain driveTrain, Shooter shooter, Limelight limelight) {
    
    addCommands(
      parallel(
       // new TrajectoryAttempt(driveTrain),
        new TurretSpool(shooter, shooterStates.HIGH)
      )
    );
  }
}

/**
 *   private final DriveTrain m_driveTrain;
  private final Limelight m_limelight;

  /** Creates a new PositionOnGoal. 
  public PositionOnGoal(DriveTrain driveTrain, Limelight limelight) {
    m_driveTrain = driveTrain;
    m_limelight = limelight;
    addRequirements(m_driveTrain, m_limelight);    // Use addRequirements() here to declare subsystem dependencies.
  }

 */
