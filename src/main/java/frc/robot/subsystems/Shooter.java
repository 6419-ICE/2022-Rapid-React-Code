// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  public static enum shooterStates {
    LOW,
    HIGH
  }

  private final WPI_TalonFX m_shooterMotor = new WPI_TalonFX(Constants.ShooterConstants.SHOOTER_MOTOR_PIN);

  private final TalonFXSensorCollection m_shooterEncoder;

  public Shooter() {
    m_shooterMotor.setNeutralMode(NeutralMode.Coast);
    m_shooterMotor.setInverted(true);

    m_shooterMotor.configOpenloopRamp(0.5);
    m_shooterMotor.configClosedloopRamp(.1);

    m_shooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    /* Config the peak and nominal outputs */
		m_shooterMotor.configNominalOutputForward(0, Constants.ShooterConstants.kTimeoutMs);
		m_shooterMotor.configNominalOutputReverse(0, Constants.ShooterConstants.kTimeoutMs);
		m_shooterMotor.configPeakOutputForward(1, Constants.ShooterConstants.kTimeoutMs);
		m_shooterMotor.configPeakOutputReverse(-1, Constants.ShooterConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		m_shooterMotor.config_kP(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kP, Constants.ShooterConstants.kTimeoutMs);
		m_shooterMotor.config_kI(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kI, Constants.ShooterConstants.kTimeoutMs);
		m_shooterMotor.config_kD(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kD, Constants.ShooterConstants.kTimeoutMs);
		m_shooterMotor.config_kF(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kF, Constants.ShooterConstants.kTimeoutMs);

    m_shooterEncoder = m_shooterMotor.getSensorCollection();
  }

  public void spoolUpHigh() {
    m_shooterMotor.set(ControlMode.Velocity, Constants.ShooterConstants.SHOOTER_HIGH_FIRING_SPEED);
  }
  
  public void spoolUpLow() {
    //m_shooterMotor.set(ControlMode.Velocity, Constants.ShooterConstants.SHOOTER_LOW_FIRING_SPEED);
    m_shooterMotor.set(ControlMode.Velocity, Constants.ShooterConstants.SHOOTER_LOW_FIRING_SPEED);
  }

  public void spoolDown(){
    m_shooterMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isShooterReadyHigh() {
    return (m_shooterEncoder.getIntegratedSensorVelocity() < -Constants.ShooterConstants.SHOOTER_HIGH_FIRING_SPEED*.9 && m_shooterEncoder.getIntegratedSensorVelocity() > -Constants.ShooterConstants.SHOOTER_HIGH_FIRING_SPEED*1.05);
  }

  public boolean isShooterReadyLow() {
    return (m_shooterEncoder.getIntegratedSensorVelocity() < -Constants.ShooterConstants.SHOOTER_LOW_FIRING_SPEED*.9);
  }
  public boolean timerReady(double init, double wait) {
    double currentTime = Timer.getFPGATimestamp();
    return currentTime > init + wait;
  }

  public double getShooterSpeed(){
    return m_shooterEncoder.getIntegratedSensorVelocity();
  }

  public void setPower(double power){
    m_shooterMotor.set(ControlMode.PercentOutput, power);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Shooter Speed", this::getShooterSpeed, null);
    builder.addBooleanProperty("Shooter Low Ready", this::isShooterReadyLow, null);
    builder.addBooleanProperty("Shooter High Ready", this::isShooterReadyHigh, null);
  }
}
