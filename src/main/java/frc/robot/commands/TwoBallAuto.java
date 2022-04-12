// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.TrajectoryPaths;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.armStates;
import frc.robot.subsystems.Shooter.shooterStates;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public TwoBallAuto(DriveTrain driveTrain, Intake intake, Uptake uptake, Shooter shooter, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      parallel(
        new AutonomousMoveIntake(intake, uptake, armStates.RAISED, uptake::isUptakeFull, 0).withTimeout(2),
    sequence(
      new WaitCommand(.5),
      parallel(
        new TrajectoryCommand(TrajectoryPaths.getTrajectoryTwoBalls2(), driveTrain),
        new TurretSpool(shooter, shooterStates.HIGH)
      ))),
      new TurretFire(shooter, uptake).withTimeout(2.5)
    );
  }
}
