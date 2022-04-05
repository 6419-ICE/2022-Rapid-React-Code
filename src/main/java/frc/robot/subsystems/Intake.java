
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {

  private final WPI_TalonSRX intakeArm = new WPI_TalonSRX(Constants.IntakeConstants.INTAKE_ARM_PIN);
  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.IntakeConstants.INTAKE_MOTOR_PIN);

  private final DigitalInput hEffectSensor = new DigitalInput(Constants.IntakeConstants.H_EFFECT_PORT);
  private final DigitalInput LmtSwitch = new DigitalInput(Constants.IntakeConstants.LMT_SWITCH_PORT);
  private final SensorCollection intakeArmSensor;

  public static enum armStates {
    RAISED,
    LOWERED,
    RAISING,
    LOWERING,
    HOMING
  };

  private armStates armState;

  private static SendableChooser<armStates> armStateChooser;

  public static enum armModes {
    MANUAL,
    MAGNETIC,
    ENCODER
  };

  private armModes armMode;

  private static SendableChooser<armModes> armModeChooser;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.setInverted(true);
    
    intakeArm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
    intakeArmSensor = intakeArm.getSensorCollection();

    intakeArm.setNeutralMode(NeutralMode.Brake);

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

  //Intake arm motor functions

  public void setIntakeArmPower(double power) {
    intakeArm.set(ControlMode.PercentOutput, power * Constants.IntakeConstants.armSpdLmt);
  }

  public void raiseIntakeArm(){
    intakeArm.set(ControlMode.PercentOutput, -1);
  }

  public void lowerIntakeArm() {
    intakeArm.set(ControlMode.PercentOutput, 1);
  }

  public void stopIntakeArm(){
    intakeArm.set(ControlMode.PercentOutput, 0);
  }

  //Intake arm sensor functions

  public int getArmRelPos(){
    return -intakeArmSensor.getQuadraturePosition();
  }
  public int getArmAbsPos(){
    return -intakeArmSensor.getPulseWidthPosition();
  }

  public void setArmRelPos(int pos){
    intakeArmSensor.setQuadraturePosition(pos, 30);
  }
  public void resetArmRelPos(){
    intakeArmSensor.setQuadraturePosition(0, 30);
  }

  public void resetArmAbsPos(){
    intakeArmSensor.setPulseWidthPosition(0, 30);
  }

  public boolean isArmBelowLowered(){
    return  this.getArmRelPos() > Constants.IntakeConstants.encoderLoweredPosition + Constants.IntakeConstants.encoderTolerance;
  }

  public boolean isArmAboveLowered(){
    return  this.getArmRelPos() < Constants.IntakeConstants.encoderLoweredPosition - Constants.IntakeConstants.encoderTolerance;
  }

  public boolean isArmBelowRaised(){
    return  this.getArmRelPos() > Constants.IntakeConstants.encoderRaisedPosition + Constants.IntakeConstants.encoderTolerance;
  }

  public boolean isArmAboveRaised(){
    return  this.getArmRelPos() < Constants.IntakeConstants.encoderRaisedPosition - Constants.IntakeConstants.encoderTolerance;
  }
  
  public boolean isArmBelowLoweredSlow(){
    return  this.getArmRelPos() > Constants.IntakeConstants.encoderLoweredPosition + 500;
  }

  public boolean isArmAboveLoweredSlow(){
    return  this.getArmRelPos() < Constants.IntakeConstants.encoderLoweredPosition - 500;
  }

  public boolean isArmBelowRaisedSlow(){
    return  this.getArmRelPos() > Constants.IntakeConstants.encoderRaisedPosition + 500;
  }

  public boolean isArmAboveRaisedSlow(){
    return  this.getArmRelPos() < Constants.IntakeConstants.encoderRaisedPosition - 500;
  }

  //Intake arm hall effect sensor functions

  public boolean getMagnetDigitalInput() {
    return !hEffectSensor.get();
  }

  public boolean getLmtSwitchInput(){
    return !LmtSwitch.get();
  }

  public armStates getCurrentArmState() {
    return armState;
  }

  public void setArmState(armStates state){
    armState = state;
  }
  
  private String returnArmState(){
    return armState.toString();
  }

  public armStates getSelectedArmState(){
    return armStateChooser.getSelected();
  }

  public armModes getCurrentArmMode() {
    return armMode;
  }

  public void setArmMode(armModes mode){
    armMode = mode;
  }
  
  private String returnArmMode(){
    return armMode.toString();
  }

  public armModes getSelectedArmMode(){
    return armModeChooser.getSelected();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putString("arm mode", this.getSelectedArmMode().toString());

    SmartDashboard.putNumber("armPos", this.getArmRelPos());
    SmartDashboard.putBoolean("limit switch", this.getLmtSwitchInput());
    //SmartDashboard.putString("armState", this.getCurrentArmState().toString());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Hall Effect", this::getMagnetDigitalInput, null);
    builder.addStringProperty("Arm State", this::returnArmState, null);
    builder.addStringProperty("Arm Mode", this::returnArmMode, null);
    builder.addDoubleProperty("Setposition", this::getArmAbsPos, null);

    armStateChooser = new SendableChooser<>();
    armStateChooser.setDefaultOption("Raised", armStates.RAISED);
    armStateChooser.addOption("Lowered", armStates.LOWERED);
    armStateChooser.addOption("Raising", armStates.RAISING);
    armStateChooser.addOption("Lowering", armStates.LOWERING);
    armStateChooser.addOption("HOMING", armStates.HOMING);

    SmartDashboard.putData("Arm State", armStateChooser);

    armModeChooser = new SendableChooser<>();
    armModeChooser.setDefaultOption("Manual", armModes.MANUAL);
    armModeChooser.addOption("Magnetic", armModes.MAGNETIC);
    armModeChooser.addOption("Encoder", armModes.ENCODER);

    SmartDashboard.putData("Arm Mode", armModeChooser);

    
    super.initSendable(builder);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends TrapezoidProfileSubsystem {

  private final WPI_TalonSRX intakeArm = new WPI_TalonSRX(Constants.IntakeConstants.INTAKE_ARM_PIN);
  private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.IntakeConstants.INTAKE_MOTOR_PIN);

  private final DigitalInput hEffectSensor = new DigitalInput(Constants.IntakeConstants.H_EFFECT_PORT);
  private final SensorCollection intakeArmEncoder;

  private final ArmFeedforward m_feedForward = new ArmFeedforward(Constants.IntakeConstants.ksVolts, Constants.IntakeConstants.kgVolts, Constants.IntakeConstants.kvVoltSecondsPerRad, Constants.IntakeConstants.kaVoltSecondsSquaredPerRad);

  public static enum armStates {
    RAISED,
    LOWERED,
    RAISING,
    LOWERING
  };

  private armStates armState;

  private static SendableChooser<armStates> armStateChooser;

  public static enum armModes {
    MANUAL,
    MAGNETIC,
    ENCODER
  };

  private armModes armMode;

  private static SendableChooser<armModes> armModeChooser;
  
  /** Creates a new Intake. 
  public Intake() {
    super(
      new Constraints(
        Constants.IntakeConstants.kMaxSpeedRadPerSecond, Constants.IntakeConstants.kMaxAccelerationRadPerSecond),
      Constants.IntakeConstants.encoderRaisedPosition
    );
    intakeMotor.setInverted(true);
    intakeArm.setNeutralMode(NeutralMode.Brake);

    intakeArm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
    intakeArm.configAllowableClosedloopError(0, Constants.IntakeConstants.encoderTolerance, 30);

    intakeArm.config_kP(0, Constants.IntakeConstants.kP);    
    intakeArm.config_kI(0, Constants.IntakeConstants.kI);
    intakeArm.config_kD(0, Constants.IntakeConstants.kD);

    intakeArmEncoder = intakeArm.getSensorCollection();

  }

  /**Getter for intake motor object 
  public TalonSRX getIntakeMotor(){
    return intakeMotor;
  }

  /**Getter for intake arm object 
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

  //Intake arm motor functions

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

  public void setArmSetpoints(double setPoint, double feedForward){
    intakeArm.set(TalonSRXControlMode.Position, setPoint, DemandType.ArbitraryFeedForward, feedForward);
  }

  //Intake arm sensor functions

  public double getArmRelPos(){
    return intakeArmEncoder.getQuadraturePosition();
  }
  public double getArmAbsPos(){
    return intakeArmEncoder.getPulseWidthPosition();
  }

  public void setArmRelPos(int pos){
    intakeArmEncoder.setQuadraturePosition(pos, 30);
  }
  public void resetArmRelPos(){
    intakeArmEncoder.setQuadraturePosition(0, 30);
  }

  public boolean isArmRaised(){
    return Math.abs(this.getArmAbsPos()-Constants.IntakeConstants.encoderRaisedPosition) < Constants.IntakeConstants.encoderTolerance;
  }

  //Intake arm hall effect sensor functions

  public boolean getMagnetDigitalInput() {
    return !hEffectSensor.get();
  }

  public armStates getCurrentArmState() {
    return armState;
  }

  public void setArmState(armStates state){
    armState = state;
  }
  
  private String returnArmState(){
    return armState.toString();
  }

  public armStates getSelectedArmState(){
    return armStateChooser.getSelected();
  }

  public armModes getCurrentArmMode() {
    return armMode;
  }

  public void setArmMode(armModes mode){
    armMode = mode;
  }
  
  private String returnArmMode(){
    return armMode.toString();
  }

  public armModes getSelectedArmMode(){
    return armModeChooser.getSelected();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected void useState(State state) {
    // TODO Auto-generated method stub
    double feedForward = m_feedForward.calculate(state.position, state.velocity);
    setArmSetpoints(state.position, feedForward);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Hall Effect", this::getMagnetDigitalInput, null);
    builder.addStringProperty("Arm State", this::returnArmState, null);
    builder.addStringProperty("Arm Mode", this::returnArmMode, null);

    armStateChooser = new SendableChooser<>();
    armStateChooser.setDefaultOption("Raised", armStates.RAISED);
    armStateChooser.addOption("Lowered", armStates.LOWERED);
    armStateChooser.addOption("Raising", armStates.RAISING);
    armStateChooser.addOption("Lowering", armStates.LOWERING);

    SmartDashboard.putData("Arm State", armStateChooser);

    armModeChooser = new SendableChooser<>();
    armModeChooser.setDefaultOption("Manual", armModes.MANUAL);
    armModeChooser.addOption("Magnetic", armModes.MAGNETIC);
    armModeChooser.addOption("Encoder", armModes.ENCODER);

    SmartDashboard.putData("Arm Mode", armModeChooser);
    
    super.initSendable(builder);
  }

}
*/