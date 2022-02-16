// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  //Drive train left side motors
  private final WPI_TalonFX[] m_leftMotors = {
    new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_FRONT_PIN),
    new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_BACK_PIN),
  };
  private final MotorControllerGroup m_leftControllerGroup = new MotorControllerGroup(m_leftMotors);


  //Drive train right side motors
  private final WPI_TalonFX[] m_rightMotors = {
    new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_FRONT_PIN),
    new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_BACK_PIN),
  };
  private final MotorControllerGroup m_rightControllerGroup = new MotorControllerGroup(m_rightMotors);

  //Differential drive train object
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftControllerGroup, m_rightControllerGroup);

  //left and right side drive encoders
  private final TalonFXSensorCollection m_leftEncoder, m_rightEncoder;

  /** Creates a new Drivetrain. */
  public DriveTrain() {
    // Sets the feedback sensor for the motors
    m_leftMotors[0].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    m_rightMotors[0].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    // Sets the encoders
    m_leftEncoder = m_leftMotors[0].getSensorCollection();
    m_rightEncoder = m_rightMotors[0].getSensorCollection();

    m_leftControllerGroup.setInverted(false);
    m_rightControllerGroup.setInverted(true);

  }

  public void drive(double l, double r){
    m_leftControllerGroup.set(l * Constants.DrivetrainConstants.speedLmt);
    m_rightControllerGroup.set(-r * Constants.DrivetrainConstants.speedLmt);
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd * Constants.DrivetrainConstants.speedLmt, rot * Constants.DrivetrainConstants.speedLmt);
  }
  
  public TalonFX getRightMotors(){
    return m_rightMotors[0];
  }

  public TalonFX getLeftMotors(){
    return m_leftMotors[0];
  }

  public void setMaxMotorSpeed(double speed) {
    m_leftMotors[0].configPeakOutputForward(speed);
    m_leftMotors[0].configPeakOutputReverse(-speed);
    m_rightMotors[0].configPeakOutputForward(speed);
    m_rightMotors[0].configPeakOutputReverse(-speed);
    m_leftMotors[1].configPeakOutputForward(speed);
    m_leftMotors[1].configPeakOutputReverse(-speed);
    m_rightMotors[1].configPeakOutputForward(speed);
    m_rightMotors[1].configPeakOutputReverse(-speed);
  }

  public void stop() {
    m_leftControllerGroup.set(0.0);
    m_rightControllerGroup.set(0.0);
  }

  public double getLeftDriveEncoderDistance() {
    return m_leftEncoder.getIntegratedSensorPosition() / Constants.DrivetrainConstants.ticksPerInch;
  }
  public double getRightDriveEncoderDistance() {
    return m_rightEncoder.getIntegratedSensorPosition() / Constants.DrivetrainConstants.ticksPerInch;
  }

  public double getAverageEncoderDistance() {
    return (getLeftDriveEncoderDistance() + getLeftDriveEncoderDistance()) / 2.0;
  }

  public double getDriveEncoderRate() {
    return ((m_leftEncoder.getIntegratedSensorVelocity() + m_rightEncoder.getIntegratedSensorVelocity()) / 2) / Constants.DrivetrainConstants.ticksPerInch * 10;
  }

  public void resetEncoders(){
    m_rightEncoder.setIntegratedSensorPosition(0, 30);
    m_leftEncoder.setIntegratedSensorPosition(0, 30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
