// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HangerConstants;

public class Hanger extends SubsystemBase {
  /** Creates a new Hanger. */

  private final WPI_TalonSRX backClimberMotors[] = {
    new WPI_TalonSRX(Constants.HangerConstants.BACK_CLIMBER_ONE_PIN),
    new WPI_TalonSRX(Constants.HangerConstants.BACK_CLIMBER_TWO_PIN),    
  };

  private final MotorControllerGroup climberGroup = new MotorControllerGroup(backClimberMotors);

  public Hanger() {
    backClimberMotors[0].setInverted(false);
    backClimberMotors[1].setInverted(true);
  }

  public void raiseHanger(){
    climberGroup.set(Constants.HangerConstants.hangerSpeedLmt);
  }  
  
  public void lowerHanger(){
    climberGroup.set(-Constants.HangerConstants.hangerSpeedLmt);
  }

  public void stopHanger(){
    climberGroup.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
