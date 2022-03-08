// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class PIDTurn extends CommandBase {

  private final DriveTrain m_driveTrain;

  private double initAngle;
  private double angle;
  private double desiredAngle;

  /** Creates a new PIDTurn. */
  public PIDTurn(DriveTrain driveTrain, double angle) {
    m_driveTrain = driveTrain;
    this.angle = angle;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetHeading();
    initAngle = m_driveTrain.getAngle();
    desiredAngle = initAngle + angle;
    m_driveTrain.setMaxMotorSpeed(Constants.DrivetrainConstants.speedLmt);
    m_driveTrain.setHeadingTarget(desiredAngle);
    m_driveTrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.drive(-m_driveTrain.headingOutput(m_driveTrain.getAngle()), m_driveTrain.headingOutput(m_driveTrain.getAngle()), Constants.DrivetrainConstants.speedLmt);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.atHeadingTarget();
  }
}
