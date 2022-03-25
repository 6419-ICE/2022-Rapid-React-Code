// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final DigitalInput hEffectSensor = new DigitalInput(Constants.HangerConstants.H_EFFECT_PORT);

  public static enum hangerStates {
    RAISED,
    LOWERED,
    RAISING,
    LOWERING
  };

  private hangerStates hangerState;

  private static SendableChooser<hangerStates> hangerStateChooser;

  public Hanger() {
    backClimberMotors[0].setInverted(false);
    backClimberMotors[1].setInverted(true);

    backClimberMotors[0].configPeakCurrentLimit(35, 10);
    backClimberMotors[0].configPeakCurrentDuration(1000, 10);
    backClimberMotors[0].configContinuousCurrentLimit(30, 10);
    backClimberMotors[0].enableCurrentLimit(true);
    backClimberMotors[1].configPeakCurrentLimit(35, 10);
    backClimberMotors[1].configPeakCurrentDuration(1000, 10);
    backClimberMotors[1].configContinuousCurrentLimit(30, 10);
    backClimberMotors[1].enableCurrentLimit(true);

    
    /*backClimberMotors[0].setNeutralMode(NeutralMode.Brake);
    backClimberMotors[1].setNeutralMode(NeutralMode.Brake);*/
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

  
  public hangerStates getCurrentHangerState() {
    return hangerState;
  }

  public void setHangerState(hangerStates state){
    hangerState = state;
  }
  
  private String returnHangerState(){
    return hangerState.toString();
  }

  public hangerStates getSelectedHangerState(){
    return hangerStateChooser.getSelected();
  }

  public boolean getMagnetDigitalInput() {
    return !hEffectSensor.get();
  }
  
  public boolean timerReady(double init, double wait) {
    double currentTime = Timer.getFPGATimestamp();
    return currentTime > init + wait;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Hall Effect", this::getMagnetDigitalInput, null);
    builder.addStringProperty("Arm State", this::returnHangerState, null);

    hangerStateChooser = new SendableChooser<>();
    hangerStateChooser.addOption("Raised", hangerStates.RAISED);
    hangerStateChooser.setDefaultOption("Lowered", hangerStates.LOWERED);
    hangerStateChooser.addOption("Raising", hangerStates.RAISING);
    hangerStateChooser.addOption("Lowering", hangerStates.LOWERING);

    SmartDashboard.putData("Hanger State", hangerStateChooser);
    
    super.initSendable(builder);
  }
}
