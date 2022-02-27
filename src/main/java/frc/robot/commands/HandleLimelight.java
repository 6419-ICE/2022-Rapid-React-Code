// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HandleLimelight extends CommandBase {
  /** Creates a new HandleLimelight. */
  private final Limelight m_limelight;
  public HandleLimelight(Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;
    addRequirements(m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Horizontal Angle", m_limelight.getHorizontalAngle());
    SmartDashboard.putNumber("Vertical Angle", m_limelight.getVerticalAngle());
    SmartDashboard.putNumber("Area", m_limelight.getTargetArea());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
