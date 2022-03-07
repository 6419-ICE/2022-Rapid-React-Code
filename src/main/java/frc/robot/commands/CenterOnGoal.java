// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;


public class CenterOnGoal extends CommandBase {
  /** Creates a new CenterOnGoal. */

  private final DriveTrain m_driveTrain;
  private final Limelight m_limelight;

  public CenterOnGoal(DriveTrain driveTrain, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_limelight = limelight;
    addRequirements(driveTrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(m_limelight.getHorizontalAngle() > 5){
        m_driveTrain.arcadeDrive(0, 1 * Constants.DrivetrainConstants.speedLmt);
      } else if(m_limelight.getHorizontalAngle() < -5){
        m_driveTrain.arcadeDrive(0, -1 * Constants.DrivetrainConstants.speedLmt);
      } else{
        m_driveTrain.arcadeDrive(0, 0);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_limelight.getHorizontalAngle()) < 5;
  }
}
