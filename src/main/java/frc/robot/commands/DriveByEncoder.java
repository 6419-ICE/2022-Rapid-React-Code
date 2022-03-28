// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.util.ISO8601DateFormat;

public class DriveByEncoder extends CommandBase {
  private double distance, speedLimit;
  private final double ticksPerInch = Constants.DrivetrainConstants.ticksPerInch;

  private final DriveTrain m_driveTrain;

  private TalonFX leftEncoder, rightEncoder;
  /** Creates a new DriveByEncoder. */

    /** Get the motors from drivetrain and set the new distance
     * @param d - The desired distance to travel by encoders
     */
    
    public DriveByEncoder(DriveTrain drivetrain, double d) {
        m_driveTrain = drivetrain;
        addRequirements(m_driveTrain);        
        distance = d;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoders();
    m_driveTrain.stop(); // Don't move on init
    m_driveTrain.setMaxMotorSpeed(1);

    m_driveTrain.resetHeading();
    m_driveTrain.setHeadingTarget(0);
    m_driveTrain.setDriveSetpoints(distance);
    SmartDashboard.putNumber("distance", distance);

}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive to set number of inches
    //m_driveTrain.drive(1 - m_driveTrain.headingOutput(m_driveTrain.getAngle(), 0), 1 + m_driveTrain.headingOutput(m_driveTrain.getAngle(), 0), Constants.DrivetrainConstants.autoSpeedLmt);

    SmartDashboard.putNumber("Left Encoder Distance", m_driveTrain.getLeftDriveEncoderDistance());
    SmartDashboard.putNumber("Right Encoder Distance", m_driveTrain.getRightDriveEncoderDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Is Interrupted", interrupted);
    //System.out.println(String.format("Drive Complete: interrupted: %s", interrupted ? "yes" : "no"));
    m_driveTrain.stop();
    m_driveTrain.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*boolean isFinished = Math.abs(m_driveTrain.getRightDriveEncoderDistance()) >= Math.abs(distance)
    || Math.abs(m_driveTrain.getLeftDriveEncoderDistance()) >= Math.abs(distance);*/
    boolean isFinished = m_driveTrain.atDriveSetpoints();
    SmartDashboard.putBoolean("CommandIsFinished", isFinished);
    return isFinished;  
  
  }
}
