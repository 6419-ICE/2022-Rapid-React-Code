// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.features2d.KAZE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
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
    double pow = Math.abs(RobotContainer.getGamepad().getRawAxis(Constants.kGamepadAxisLeftStickX)) > .05 ? RobotContainer.getGamepad().getRawAxis(Constants.kGamepadAxisLeftStickX) : 0;
    double turn = Math.abs(RobotContainer.getGamepad().getRawAxis(Constants.kGamepadAxisRightStickX))> .05 ? RobotContainer.getGamepad().getRawAxis(Constants.kGamepadAxisRightStickX) : 0;
    m_driveTrain.arcadeDrive(pow, turn);
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
