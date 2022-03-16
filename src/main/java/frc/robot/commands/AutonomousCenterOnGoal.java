// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.*;


public class AutonomousCenterOnGoal extends CommandBase {
  /** Creates a new CenterOnGoal. */

  private final DriveTrain m_driveTrain;
  private final Limelight m_limelight;

  private double initAngle;
  private double angle;
  private double desiredAngle;

  public AutonomousCenterOnGoal(DriveTrain driveTrain, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_limelight = limelight;
    addRequirements(driveTrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredAngle = initAngle + angle;
    m_driveTrain.setMaxMotorSpeed(Constants.DrivetrainConstants.speedLmt);
    m_driveTrain.setHeadingTarget(desiredAngle);
    m_driveTrain.stop();
    // m_limelight.setCameraMode(CameraMode.VISION);
    // m_limelight.setLightMode(LightMode.ON);
    SmartDashboard.putBoolean("Finished", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double frictionCoefficient = 0.07;
      double turnPower = ((Math.abs(m_limelight.getHorizontalAngle()) * (1/27.5)) * Constants.DrivetrainConstants.speedLmt) + frictionCoefficient;
      SmartDashboard.putNumber("Angle", m_limelight.getHorizontalAngle());
      if(m_limelight.getHorizontalAngle() > 1.5){
        m_driveTrain.arcadeDrive(0, turnPower);
        SmartDashboard.putString("Direction", "Right");
        SmartDashboard.putNumber("Power", turnPower);
      } else if(m_limelight.getHorizontalAngle() < -1.5){
        turnPower = -turnPower;
        m_driveTrain.arcadeDrive(0, turnPower);
        SmartDashboard.putString("Direction", "Left");
        SmartDashboard.putNumber("Power", turnPower);
      } else{
        m_driveTrain.arcadeDrive(0, 0);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Finished", true);
    // m_limelight.setCameraMode(CameraMode.DRIVER_CAMERA);
    // m_limelight.setLightMode(LightMode.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_limelight.getHorizontalAngle()) < 1.5;
  }
}
