// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.shooterStates;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpoolUpFire extends SequentialCommandGroup {
  /** Creates a new SpoolUpFire. */
  public SpoolUpFire(Uptake uptake, Shooter shooter, shooterStates shooterState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TurretSpool(shooter, shooterState),
      new TurretFire(shooter, uptake, RobotContainer::getShooterButton)
    );
  }
}
