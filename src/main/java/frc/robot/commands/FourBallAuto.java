// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.TrajectoryPaths;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.armStates;
import frc.robot.subsystems.Shooter.shooterStates;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallAuto extends SequentialCommandGroup {
  /** Creates a new FourBallAuto. */
  public FourBallAuto(DriveTrain driveTrain, Intake intake, Uptake uptake, Shooter shooter, Limelight limelight) {
    driveTrain.resetHeading();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());paths/output/%s.wpilib.jsonpaths/output/
    addCommands(
      parallel(
        new TrajectoryCommand(TrajectoryPaths.getTrajectoryFromPath("paths/output/start-to-firstball.wpilib.json"), driveTrain),
        new AutonomousMoveIntake(intake, armStates.RAISED),
        new TurretSpool(shooter, shooterStates.HIGH)
      ),
      new TurretFire(shooter, uptake).withTimeout(2.5),
      //new WaitCommand(2.5),
      parallel(new TrajectoryCommand(TrajectoryPaths.getTrajectoryFromPath("paths/output/firstball-to-terminal.wpilib.json"), driveTrain),
      new RunCommand(() -> {if(Math.abs(intake.getArmAbsPos())-Math.abs(Constants.IntakeConstants.encoderLoweredPosition) > 500){
        intake.raiseIntakeArm();
    
      }else if(Math.abs(Constants.IntakeConstants.encoderLoweredPosition)-Math.abs(intake.getArmAbsPos()) > 500){
        intake.lowerIntakeArm();
      }else{
        intake.stopIntakeArm();
      }  }, intake).withTimeout(3)
      ),
      new RunCommand(() -> {intake.runIntakeMotor(); uptake.runUptake();}, intake, uptake).withTimeout(2),
      parallel(
        new TrajectoryCommand(TrajectoryPaths.getTrajectoryFromPath("paths/output/terminal-to-firstball.wpilib.json"), driveTrain),
        new TurretSpool(shooter, shooterStates.HIGH)
      ),
      new TurretFire(shooter, uptake)
    );
  }
}
