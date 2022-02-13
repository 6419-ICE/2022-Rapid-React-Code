// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.features2d.KAZE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveTrain;

public class HandleDriveTrain extends CommandBase {
  /** Creates a new HandleDrivetrain. */

  private final DriveTrain m_driveTrain;

  public HandleDriveTrain(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    addRequirements(m_driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.drive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = Utilities.applyDeadband(-RobotContainer.getGamepad().getRawAxis(Constants.kGamepadAxisLeftStickX), 0.03);
    double turn = Utilities.applyDeadband(RobotContainer.getGamepad().getRawAxis(Constants.kGamepadAxisRightStickX), 0.03);
    power = Math.copySign(Math.abs(Math.pow(power, 2)), power);
    turn = Math.copySign(Math.abs(Math.pow(turn, 2)), turn);
    m_driveTrain.arcadeDrive(power, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
