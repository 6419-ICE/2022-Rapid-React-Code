// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final TalonSRX intakeArm = new TalonSRX(Constants.IntakeConstants.INTAKE_ARM_PIN);
  private final TalonSRX intakeMotor = new TalonSRX(Constants.IntakeConstants.INTAKE_MOTOR_PIN);

  private final DigitalInput hEffectSensor = new DigitalInput(Constants.IntakeConstants.H_EFFECT_PORT);
  
  /** Creates a new Intake. */
  public Intake() {

  }

  /**Getter for intake motor object */
  public TalonSRX getIntakeMotor(){
    return intakeMotor;
  }

  /**Getter for intake arm object */
  public TalonSRX getIntakeArm(){
    return intakeArm;
  }

  //Intake motor functions
  
  public void setIntakeMotorPower(double power) {
    intakeMotor.set(ControlMode.PercentOutput, power * Constants.IntakeConstants.motorSpdLmt);
  }

  public void runIntakeMotor(){
    intakeMotor.set(ControlMode.PercentOutput, 1 * Constants.IntakeConstants.motorSpdLmt);
  }

  public void reverseIntakeMotor() {
    intakeMotor.set(ControlMode.PercentOutput, -1 * Constants.IntakeConstants.motorSpdLmt);
  }

  public void stopIntakeMotor(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  //Intake arm functions

  public void setIntakeArmPower(double power) {
    intakeArm.set(ControlMode.PercentOutput, power * Constants.IntakeConstants.armSpdLmt);
  }

  public void raiseIntakeArm(){
    intakeArm.set(ControlMode.PercentOutput, -1 * Constants.IntakeConstants.armSpdLmt);
  }

  public void lowerIntakeArm() {
    intakeArm.set(ControlMode.PercentOutput, 1 * Constants.IntakeConstants.armSpdLmt);
  }

  public void stopIntakeArm(){
    intakeArm.set(ControlMode.PercentOutput, 0);
  }

  public boolean getMagnetDigitalInput() {
    return !hEffectSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Hall Effect", this::getMagnetDigitalInput, null);
  }
}
