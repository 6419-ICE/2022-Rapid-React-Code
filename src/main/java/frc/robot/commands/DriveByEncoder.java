// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import static frc.robot.RobotContainer.m_driveTrain;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class DriveByEncoder extends CommandBase {
  private double distance, speedLimit;
  private final double ticksPerInch = Constants.DrivetrainConstants.ticksPerInch;

  private TalonFX leftEncoder, rightEncoder;
  /** Creates a new DriveByEncoder. */

    /** Get the motors from drivetrain and set the new distance
     * @param d - The desired distance to travel by encoders
     */
    public DriveByEncoder(double d) {
        addRequirements(m_driveTrain);        
        this.speedLimit = Constants.DrivetrainConstants.speedLmt;
        leftEncoder = m_driveTrain.getLeftMotors();
        rightEncoder = m_driveTrain.getRightMotors();

        distance = d;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveByEncoder Initialized.");

    m_driveTrain.resetEncoders();
    m_driveTrain.setMaxMotorSpeed(.3);
    m_driveTrain.stop(); // Don't move on init
    //drivetrain.setSetpoints(distance, distance);
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive to set number of inches
    if(distance < 0){
      m_driveTrain.drive(0.5, 0.5);
    } else{
      m_driveTrain.drive(-0.5, -0.5);
    }
    SmartDashboard.putNumber("Left Encoder Distance", m_driveTrain.getLeftDriveEncoderDistance());
    SmartDashboard.putNumber("Right Encoder Distance", m_driveTrain.getRightDriveEncoderDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(String.format("Drive Complete: interrupted: %s", interrupted ? "yes" : "no"));
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveTrain.getRightDriveEncoderDistance() / ticksPerInch) >= Math.abs(distance)
    || Math.abs(m_driveTrain.getLeftDriveEncoderDistance() / ticksPerInch) >= Math.abs(distance);  }
}
