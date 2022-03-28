// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public class DriveTrain extends SubsystemBase {

  //Drive train left side motors
  private final WPI_TalonFX[] m_leftMotors = {
    new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_FRONT_PIN),
    new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_BACK_PIN),
  };
  private final WPI_TalonFX m_leftMotorLead = m_leftMotors[0];

  private final TalonFXInvertType m_leftInvert = TalonFXInvertType.CounterClockwise;
  private final TalonFXConfiguration m_leftConfig0 = new TalonFXConfiguration();
  private final TalonFXSensorCollection m_leftEncoder;

  private final MotorControllerGroup m_leftControllerGroup = new MotorControllerGroup(m_leftMotors);


  //Drive train right side motors
  private final WPI_TalonFX[] m_rightMotors = {
    new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_FRONT_PIN),
    new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_BACK_PIN),
  };
  private final WPI_TalonFX m_rightMotorLead = m_rightMotors[0];

  private final TalonFXInvertType m_rightInvert = TalonFXInvertType.Clockwise;
  private final TalonFXConfiguration m_rightConfig0 = new TalonFXConfiguration();
  private final TalonFXSensorCollection m_rightEncoder;

  private final MotorControllerGroup m_rightControllerGroup = new MotorControllerGroup(m_rightMotors);

  //Differential drive train object
  private final DifferentialDrive m_drive;

  private final DifferentialDriveOdometry m_odometry;

  private PIDController headingPIDController;

  //NavX gyro
  private final AHRS m_gyro;

  /** Creates a new Drivetrain. */
  public DriveTrain() {
    /*
    m_leftMotorLead.setSensorPhase(false);
    m_rightMotorLead.setSensorPhase(true);*/

    m_leftControllerGroup.setInverted(false);
    m_rightControllerGroup.setInverted(false);

    primaryConfig();

    m_leftMotorLead.configAllSettings(m_leftConfig0);
		m_rightMotorLead.configAllSettings(m_rightConfig0);
    m_leftMotors[1].configAllSettings(m_leftConfig0);
		m_rightMotors[1].configAllSettings(m_rightConfig0);
    
    // Sets the encoders
    m_leftEncoder = m_leftMotorLead.getSensorCollection();
    m_rightEncoder = m_rightMotorLead.getSensorCollection();

    m_leftEncoder.setIntegratedSensorPosition(0, 30);
    m_rightEncoder.setIntegratedSensorPosition(0, 30);

    m_leftMotorLead.setSelectedSensorPosition(0);
    m_rightMotorLead.setSelectedSensorPosition(0);

    m_leftMotorLead.selectProfileSlot(0, 0);
    m_rightMotorLead.selectProfileSlot(0, 0);

    m_leftMotorLead.setInverted(m_leftInvert);
    m_leftMotors[1].setInverted(m_leftInvert);
    m_rightMotorLead.setInverted(m_rightInvert);
    m_rightMotors[1].setInverted(m_rightInvert);

    setMotorLeader();

    headingPIDController = new PIDController(
      Constants.DrivetrainConstants.HeadingPID.kP, 
      Constants.DrivetrainConstants.HeadingPID.kI, 
      Constants.DrivetrainConstants.HeadingPID.kD
    );

    headingPIDController.setTolerance(Constants.DrivetrainConstants.HeadingPID.headingPIDTolerance);
    m_gyro = new AHRS(SerialPort.Port.kMXP);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
    m_drive = new DifferentialDrive(m_leftControllerGroup, m_rightControllerGroup);
    /*
    m_rightMotorLead.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, 30);
		m_rightMotorLead.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, 30);
		m_rightMotorLead.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, 30);
		m_leftMotorLead.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 30);*/
    
    /*
    m_leftMotorLead.setInverted(false);
    m_rightMotorLead.setInverted(true);
    m_leftMotors[1].setInverted(false);
    m_rightMotors[1].setInverted(true);*
    // Sets the feedback sensor for the motors
    m_leftMotorLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    m_rightMotorLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    m_leftMotorLead.configAllowableClosedloopError(0, 0, 30);
    m_rightMotorLead.configAllowableClosedloopError(0, 0, 30);

    m_leftMotorLead.configClosedloopRamp(0.1);
    m_rightMotorLead.configClosedloopRamp(0.1);

    /* Config PID values: Config 0 
    m_leftMotorLead.config_kP(0, Constants.DrivetrainConstants.DrivePID.kP); // 0 is the slot index for this current PID config
    m_leftMotorLead.config_kI(0, Constants.DrivetrainConstants.DrivePID.kI);
    m_leftMotorLead.config_kD(0, Constants.DrivetrainConstants.DrivePID.kD);
    m_leftMotorLead.config_kF(0, Constants.DrivetrainConstants.DrivePID.kF);

    m_rightMotorLead.config_kP(0, Constants.DrivetrainConstants.DrivePID.kP); // 0 is the slot index for this current PID config
    m_rightMotorLead.config_kI(0, Constants.DrivetrainConstants.DrivePID.kI);
    m_rightMotorLead.config_kD(0, Constants.DrivetrainConstants.DrivePID.kD);
    m_rightMotorLead.config_kF(0, Constants.DrivetrainConstants.DrivePID.kF);*/


  }
  
  public TalonFX getLeftMotors(){
    return m_leftMotorLead;
  }

  public TalonFX getRightMotors(){
    return m_rightMotorLead;
  }

  public void drive(double l, double r){
    m_leftControllerGroup.set(l * Constants.DrivetrainConstants.speedLmt);
    m_rightControllerGroup.set(r * Constants.DrivetrainConstants.speedLmt);
  }

  public void drive(double l, double r, double speedLmt){
    m_leftControllerGroup.set(l * speedLmt);
    m_rightControllerGroup.set(r * speedLmt);
  }


  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftControllerGroup.setVoltage(leftVolts);
    m_rightControllerGroup.setVoltage(rightVolts);

    m_drive.feed();
  }

  public void stop() {
    m_leftControllerGroup.set(0.0);
    m_rightControllerGroup.set(0.0);
  }

  public void setMaxMotorSpeed(TalonFXConfiguration config1, TalonFXConfiguration config2, double speed) {
    config1.peakOutputForward = speed;
    config1.peakOutputReverse = -speed;
    config2.peakOutputForward = speed;
    config2.peakOutputReverse = -speed;

  }

  public void setMaxMotorSpeed(double speed) {
    m_leftConfig0.peakOutputForward = speed;
    m_leftConfig0.peakOutputReverse = -speed;
    m_rightConfig0.peakOutputForward = speed;
    m_rightConfig0.peakOutputReverse = -speed;

    m_leftMotorLead.configAllSettings(m_leftConfig0);
    m_rightMotorLead.configAllSettings(m_rightConfig0);
    m_leftMotors[1].configAllSettings(m_leftConfig0);
		m_rightMotors[1].configAllSettings(m_rightConfig0);
  }

  public void setMinMotorSpeed(TalonFXConfiguration config1, TalonFXConfiguration config2, double speed) {
    config1.nominalOutputForward = speed;
    config1.nominalOutputReverse = -speed;
    config2.nominalOutputForward = speed;
    config2.nominalOutputReverse = -speed;

  }

  public void setMotorNeutralMode(NeutralMode neutralMode){
    m_leftMotors[0].setNeutralMode(neutralMode);
    m_leftMotors[1].setNeutralMode(neutralMode);
    m_rightMotors[0].setNeutralMode(neutralMode);
    m_rightMotors[1].setNeutralMode(neutralMode);
  }


  public void setMotorLeader() {
    m_leftMotors[1].follow(m_leftMotorLead);
    m_rightMotors[1].follow(m_rightMotorLead);
  }

  public void primaryConfig(){
    m_leftConfig0.primaryPID.selectedFeedbackSensor =  TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    m_rightConfig0.primaryPID.selectedFeedbackSensor =  TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    m_leftConfig0.closedloopRamp = 0.1;
    m_rightConfig0.closedloopRamp = 0.1;

    /*m_rightConfig0.remoteFilter1.remoteSensorDeviceID = m_leftMotorLead.getDeviceID();
    m_rightConfig0.remoteFilter1.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type*/

   // setRobotTurnConfigs(m_rightInvert, m_rightConfig0); 
    
    setMaxMotorSpeed(m_rightConfig0, m_leftConfig0, Constants.DrivetrainConstants.speedLmt);
    setMinMotorSpeed(m_rightConfig0, m_leftConfig0, 0);
    
    setConfigPIDSlot(m_leftConfig0.slot0, 
      Constants.DrivetrainConstants.DrivePID.kP, 
      Constants.DrivetrainConstants.DrivePID.kI, 
      Constants.DrivetrainConstants.DrivePID.kD, 
      Constants.DrivetrainConstants.DrivePID.kF);
    setConfigPIDSlot(m_rightConfig0.slot0, 
      Constants.DrivetrainConstants.DrivePID.kP, 
      Constants.DrivetrainConstants.DrivePID.kI, 
      Constants.DrivetrainConstants.DrivePID.kD, 
      Constants.DrivetrainConstants.DrivePID.kF);

    setConfigPIDSlot(m_rightConfig0.slot1, 
      Constants.DrivetrainConstants.AuxPID.kP, 
      Constants.DrivetrainConstants.AuxPID.kI, 
      Constants.DrivetrainConstants.AuxPID.kD, 
      Constants.DrivetrainConstants.AuxPID.kF);
    
    
  }

  void setRobotTurnConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot heading.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   heading magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.
				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.
				Will a sensor sum or difference give us a positive heading?
				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.
					Phase | Term 0   |   Term 1  | Result
				Sum:  -((-)Master + (+)Aux   )| OK - magnitude will cancel each other out
				Diff: -((-)Master - (+)Aux   )| NOT OK - magnitude increases with forward distance.
				Diff: -((+)Aux    - (-)Master)| NOT OK - magnitude decreases with forward distance
			*/

			masterConfig.sum0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1

			/*
				PID Polarity
				With the sensor phasing taken care of, we now need to determine if the PID polarity is in the correct direction
				This is important because if the PID polarity is incorrect, we will run away while trying to correct
				Will inverting the polarity give us a positive counterclockwise heading?
				If we're moving counterclockwise(+), and the master is on the right side and inverted,
				it will have a negative velocity and the auxiliary will have a negative velocity
				 heading = right + left
				 heading = (-) + (-)
				 heading = (-)
				Let's assume a setpoint of 0 heading.
				This produces a positive error, in order to cancel the error, the right master needs to
				drive backwards. This means the PID polarity needs to be inverted to handle this
				
				Conversely, if we're moving counterclwise(+), and the master is on the left side and inverted,
				it will have a positive velocity and the auxiliary will have a positive velocity.
				 heading = right + left
				 heading = (+) + (+)
				 heading = (+)
				Let's assume a setpoint of 0 heading.
				This produces a negative error, in order to cancel the error, the left master needs to
				drive forwards. This means the PID polarity needs to be inverted to handle this
			*/
			masterConfig.auxPIDPolarity = true;
		} else {
			/* Master is not inverted, both sides are positive so we can diff them. */
			masterConfig.diff0Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Sum0 + Sum1
			/* With current diff terms, a counterclockwise rotation results in negative heading with a right master */
			masterConfig.auxPIDPolarity = true;
		}
		/**
		 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
		 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
		 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
		 *  ... so at 3600 units per 360', that ensures 0.1 degree precision in firmware closed-loop
		 *  and motion profile trajectory points can range +-2 rotations.
		 */
		//masterConfig.auxiliaryPID.selectedFeedbackCoefficient = Constants.kTurnTravelUnitsPerRotation / Constants.kEncoderUnitsPerRotation;
	}

  public void setConfigPIDSlot(SlotConfiguration slot, double kP, double kI, double kD, double kF){
    slot.kP = kP;
    slot.kI = kI;
    slot.kD = kD;
    slot.kF = kF;
    slot.closedLoopPeakOutput = 1.0;
    slot.closedLoopPeriod = 1;
  }

  public TalonFXSensorCollection getLeftEncoder() {
    return m_leftEncoder;
  }

  public TalonFXSensorCollection getRightEncoder() {
    return m_rightEncoder;
  }

  public double getLeftDriveEncoderTick() {
    return m_leftEncoder.getIntegratedSensorPosition();
  }
  public double getRightDriveEncoderTick() {
    return  -m_rightEncoder.getIntegratedSensorPosition();
  }

  public double getLeftDriveEncoderDistance() {
    return getLeftDriveEncoderTick()/ Constants.DrivetrainConstants.ticksPerInch;
  }

  public double getRightDriveEncoderDistance() {
    return getRightDriveEncoderTick()/ Constants.DrivetrainConstants.ticksPerInch;
  }

  public double getLeftDriveEncoderMeters() {
    return getLeftDriveEncoderTick()/ Constants.DrivetrainConstants.ticksPerInch * 0.0254;
  }

  public double getRightDriveEncoderMeters() {
    return getRightDriveEncoderTick()/ Constants.DrivetrainConstants.ticksPerInch * 0.0254;
  }

  public double getAverageEncoderDistance() {
    return (getLeftDriveEncoderDistance() + getLeftDriveEncoderDistance()) / 2.0;
  }

  public double getRightDriveEncoderRate() {
    return -m_rightEncoder.getIntegratedSensorVelocity();
  }

  public double getLeftDriveEncoderRate() {
    return m_leftEncoder.getIntegratedSensorVelocity();
  }

  public double getDriveEncoderRate() {
    return ((m_leftEncoder.getIntegratedSensorVelocity() + m_rightEncoder.getIntegratedSensorVelocity()) / 2) / Constants.DrivetrainConstants.ticksPerInch * 10;
  }

  public void resetEncoders(){
    m_rightEncoder.setIntegratedSensorPosition(0, 30);
    m_leftEncoder.setIntegratedSensorPosition(0, 30);
  }

  public double getAngle(){
    return m_gyro.getAngle();
  }

  public Rotation2d getRotation2d(){
    return m_gyro.getRotation2d();
  }

  public double getYaw(){
    return m_gyro.getYaw();
  }

  public double getPitch(){
    return m_gyro.getPitch();
  }

  public double getRoll(){
    return m_gyro.getRoll();
  }

  public double getTurnRate(){
    return -m_gyro.getRate();
  }

  public void resetHeading(){
    m_gyro.reset();
    m_gyro.zeroYaw();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  public double getDegrees(){
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }

  public double getXPos(){
    return m_odometry.getPoseMeters().getX();
  }

  public double getYPos(){
    return m_odometry.getPoseMeters().getY();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds((getLeftDriveEncoderRate() * 10) / (Constants.DrivetrainConstants.ticksPerInch / .0254), (getRightDriveEncoderRate() * 10) / (Constants.DrivetrainConstants.ticksPerInch / .0254));
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getYaw()));
  }

  public void setDriveSetpoints(double distance){
/*
    m_rightMotorLead.selectProfileSlot(0, 0);
    m_rightMotorLead.selectProfileSlot(1, 1);

    setAuxPidMode(distance, m_rightMotorLead.getSelectedSensorPosition(1));
    
    m_leftMotorLead.set(TalonFXControlMode.Position, left * Constants.DrivetrainConstants.ticksPerInch);
    m_rightMotorLead.set(TalonFXControlMode.Position, right * Constants.DrivetrainConstants.ticksPerInch);
    m_leftMotors[1].set(TalonFXControlMode.Position, left * Constants.DrivetrainConstants.ticksPerInch);
    m_rightMotors[1].set(TalonFXControlMode.Position, right * Constants.DrivetrainConstants.ticksPerInch);
    */
  }

  public double getDriveSetpoints(){
    return m_rightMotorLead.getClosedLoopTarget(0);
  }

  public boolean atDriveSetpoints(){
    return (
      (m_leftMotorLead.getClosedLoopError() < Constants.DrivetrainConstants.DrivePID.drivePIDTolerance * Constants.DrivetrainConstants.ticksPerInch &&
      m_leftMotorLead.getClosedLoopError() > -Constants.DrivetrainConstants.DrivePID.drivePIDTolerance * Constants.DrivetrainConstants.ticksPerInch) &&
      (m_rightMotorLead.getClosedLoopError() < Constants.DrivetrainConstants.DrivePID.drivePIDTolerance * Constants.DrivetrainConstants.ticksPerInch &&
      m_rightMotorLead.getClosedLoopError() > -Constants.DrivetrainConstants.DrivePID.drivePIDTolerance * Constants.DrivetrainConstants.ticksPerInch)
    );
    //return m_rightMotorLead.getClosedLoopError(0)
  }

  public void setAuxPidMode(double targetDistance, double targetAngle){
    m_rightMotorLead.set(TalonFXControlMode.Position, targetDistance * 0.025400013716 * Constants.DrivetrainConstants.ticksPerInch , DemandType.AuxPID, targetAngle);
    m_leftMotorLead.follow(m_rightMotorLead, FollowerType.AuxOutput1);
  }

  public void setHeadingTarget(double target){
    headingPIDController.setSetpoint(target);
  }

  public double getHeadingTarget() {
    return headingPIDController.getSetpoint();
  }

  public boolean atHeadingTarget() {
    return headingPIDController.atSetpoint();
  }

  public double headingOutput(double value){
    double clampedValue = MathUtil.clamp(
                            Math.abs(headingPIDController.calculate(value)),
                            Constants.DrivetrainConstants.HeadingPID.lowerClampBoundary, 
                            Constants.DrivetrainConstants.HeadingPID.upperClampBoundary);

    return Math.copySign(clampedValue, value);
  }
  public double headingOutputNoClamp(double value){
    /*double clampedValue = MathUtil.clamp(
                            Math.abs(headingPIDController.calculate(value)),
                            Constants.DrivetrainConstants.HeadingPID.lowerClampBoundary, 
                            Constants.DrivetrainConstants.HeadingPID.upperClampBoundary);*/
    return headingPIDController.calculate(value);
  }

  public double headingOutput(double value, double setPoint){
    double clampedValue = MathUtil.clamp(
                            Math.abs(headingPIDController.calculate(value, setPoint)),
                            Constants.DrivetrainConstants.HeadingPID.lowerClampBoundary, 
                            Constants.DrivetrainConstants.HeadingPID.upperClampBoundary);

    return Math.copySign(clampedValue, value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getYaw()), getLeftDriveEncoderMeters(), getRightDriveEncoderMeters());
    SmartDashboard.putNumber("leftSpeed", getWheelSpeeds().leftMetersPerSecond);

    SmartDashboard.putNumber("rightSpeed", getWheelSpeeds().rightMetersPerSecond);

  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("Heading", this::getAngle, null);
      builder.addDoubleProperty("Yaw", this::getYaw, null);
      builder.addDoubleProperty("Degrees", this::getDegrees, null);
      builder.addDoubleProperty("Left Encoder Distance", this::getLeftDriveEncoderMeters, null);
      builder.addDoubleProperty("Right Encoder Distance", this::getRightDriveEncoderMeters, null);
      builder.addDoubleProperty("X Position", this::getXPos, null);
      builder.addDoubleProperty("Y Position", this::getYPos, null);

      super.initSendable(builder);
  }
}
