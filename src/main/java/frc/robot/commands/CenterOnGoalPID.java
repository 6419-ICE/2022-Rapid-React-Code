// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class CenterOnGoalPID extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final Limelight m_limelight;

  private double initAngle;
  private double angle;
  private double desiredAngle;

  private boolean tooClose;

  /** Creates a new CenterOnGoalPID. */
  public CenterOnGoalPID(DriveTrain driveTrain, Limelight limelight) {
    m_driveTrain = driveTrain;
    m_limelight = limelight;
    addRequirements(m_driveTrain, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetHeading();
    initAngle = m_driveTrain.getAngle();
    desiredAngle = initAngle + m_limelight.getHorizontalAngle();
    tooClose = Math.abs(m_limelight.getHorizontalAngle()) < 12;
    m_driveTrain.setMaxMotorSpeed(Constants.DrivetrainConstants.autoSpeedLmt);
    m_driveTrain.setHeadingTarget(desiredAngle);
    m_driveTrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_driveTrain.drive(-Constants.DrivetrainConstants.HeadingPID.lowerClampBoundary, Constants.DrivetrainConstants.HeadingPID.lowerClampBoundary, 1);
    if(tooClose && Math.abs(m_limelight.getHorizontalAngle()) < 12){
      m_driveTrain.arcadeDrive(0, Math.copySign(.3, m_limelight.getHorizontalAngle()));
    } else if(tooClose && Math.abs(m_limelight.getHorizontalAngle()) > 12){
      tooClose = false;
    } else if(!tooClose) {
      m_driveTrain.drive(-m_driveTrain.headingOutputNoClamp(m_driveTrain.getAngle()), m_driveTrain.headingOutputNoClamp(m_driveTrain.getAngle()), 1);
    }
    SmartDashboard.putBoolean("too close", tooClose);
     // m_driveTrain.drive(-m_driveTrain.headingOutputNoClamp(m_driveTrain.getAngle()), m_driveTrain.headingOutputNoClamp(m_driveTrain.getAngle()), 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setMaxMotorSpeed(Constants.DrivetrainConstants.speedLmt);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.atHeadingTarget();
  }
}
