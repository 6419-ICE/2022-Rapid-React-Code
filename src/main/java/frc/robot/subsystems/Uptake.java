// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Uptake extends SubsystemBase {
  /** Creates a new Uptake. */

  private final TalonSRX uptakeMotor = new TalonSRX(Constants.UptakeConstants.UPTAKE_MOTOR_PIN);
  private final CANSparkMax loaderMotor = new CANSparkMax(Constants.UptakeConstants.LOADER_MOTOR_PIN,  CANSparkMaxLowLevel.MotorType.kBrushless);

  private final DigitalInput loadSensor = new DigitalInput(Constants.UptakeConstants.LOAD_SENSOR_PORT);

  public Uptake() {
    uptakeMotor.setNeutralMode(NeutralMode.Brake);
    loaderMotor.setIdleMode(IdleMode.kBrake);

    loaderMotor.setInverted(true);

    uptakeMotor.configPeakOutputForward(Constants.UptakeConstants.uptakeMotorLmt);
    uptakeMotor.configPeakOutputReverse(-Constants.UptakeConstants.uptakeMotorLmt);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLoaderPower(double power) {
    loaderMotor.set(power * Constants.UptakeConstants.loadMotorLmt);
  }

  public void runLoader(){
    loaderMotor.set(1 * Constants.UptakeConstants.loadMotorLmt);
  }

  public void reverseLoader(){
    loaderMotor.set(-1 * Constants.UptakeConstants.loadMotorLmt);
  }

  public void stopLoader(){
    loaderMotor.set(0);
  }

  public void setUptakePower(double power) {
    uptakeMotor.set(ControlMode.PercentOutput, power);
  }

  public void runUptake(){
    uptakeMotor.set(ControlMode.PercentOutput, .7);
  }

  public void reverseUptake(){
    uptakeMotor.set(ControlMode.PercentOutput, -.7);
  }

  public void stopUptake(){
    uptakeMotor.set(ControlMode.PercentOutput, 0);
  }


  public boolean isCargoPresent() {
    return !loadSensor.get();
  }
}
