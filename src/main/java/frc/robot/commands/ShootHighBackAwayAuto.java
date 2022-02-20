// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Shooter.shooterStates;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootHighBackAwayAuto extends SequentialCommandGroup {
  /** Creates a new ShootLowBackAway. */
  public ShootHighBackAwayAuto() {
    Shooter shooter = RobotContainer.m_shooter;
    Uptake uptake = RobotContainer.m_uptake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Back up enough to shoot into the high goal
      new DriveByEncoder(-36),

      // Shoot into the low goal
      new AutonomousShoot(uptake, shooter, shooterStates.HIGH, 10000),

      // Back out of the line 
      new DriveByEncoder(-36)
      
    );
  }
}