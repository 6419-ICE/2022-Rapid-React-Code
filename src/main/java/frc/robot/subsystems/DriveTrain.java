// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.AHRS;

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
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  //Drive train left side motors
  private final WPI_TalonFX[] m_leftMotors = {
    new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_FRONT_PIN),
    new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_BACK_PIN),
  };
  private final WPI_TalonFX m_leftMotorLead = m_leftMotors[0];
  private final MotorControllerGroup m_leftControllerGroup = new MotorControllerGroup(m_leftMotors);


  //Drive train right side motors
  private final WPI_TalonFX[] m_rightMotors = {
    new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_FRONT_PIN),
    new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_BACK_PIN),
  };
  private final WPI_TalonFX m_rightMotorLead = m_rightMotors[0];
  private final MotorControllerGroup m_rightControllerGroup = new MotorControllerGroup(m_rightMotors);

  //Differential drive train object
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftControllerGroup, m_rightControllerGroup);

  private final DifferentialDriveOdometry m_odometry;

  //left and right side drive encoders
  private final TalonFXSensorCollection m_leftEncoder, m_rightEncoder;

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
    
    m_leftMotorLead.setInverted(false);
    m_rightMotorLead.setInverted(true);
    m_leftMotors[1].setInverted(false);
    m_rightMotors[1].setInverted(true);
    // Sets the feedback sensor for the motors
    m_leftMotorLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    m_rightMotorLead.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    m_leftMotorLead.configAllowableClosedloopError(0, 0, 30);
    m_rightMotorLead.configAllowableClosedloopError(0, 0, 30);

    m_leftMotorLead.configClosedloopRamp(0.1);
    m_rightMotorLead.configClosedloopRamp(0.1);

    /* Config PID values: Config 0 */
    m_leftMotorLead.config_kP(0, Constants.DrivetrainConstants.DrivePID.kP); // 0 is the slot index for this current PID config
    m_leftMotorLead.config_kI(0, Constants.DrivetrainConstants.DrivePID.kI);
    m_leftMotorLead.config_kD(0, Constants.DrivetrainConstants.DrivePID.kD);
    m_leftMotorLead.config_kF(0, Constants.DrivetrainConstants.DrivePID.kF);

    m_rightMotorLead.config_kP(0, Constants.DrivetrainConstants.DrivePID.kP); // 0 is the slot index for this current PID config
    m_rightMotorLead.config_kI(0, Constants.DrivetrainConstants.DrivePID.kI);
    m_rightMotorLead.config_kD(0, Constants.DrivetrainConstants.DrivePID.kD);
    m_rightMotorLead.config_kF(0, Constants.DrivetrainConstants.DrivePID.kF);

    // Sets the encoders
    m_leftEncoder = m_leftMotorLead.getSensorCollection();
    m_rightEncoder = m_rightMotorLead.getSensorCollection();

    m_leftEncoder.setIntegratedSensorPosition(0, 30);
    m_rightEncoder.setIntegratedSensorPosition(0, 30);

    m_leftMotorLead.setSelectedSensorPosition(0);
    m_rightMotorLead.setSelectedSensorPosition(0);

    setMaxMotorSpeed(Constants.DrivetrainConstants.speedLmt);
    setMotorNeutralMode(NeutralMode.Brake);
    setMotorLeader();

    headingPIDController = new PIDController(
      Constants.DrivetrainConstants.HeadingPID.kP, 
      Constants.DrivetrainConstants.HeadingPID.kI, 
      Constants.DrivetrainConstants.HeadingPID.kD
    );

    headingPIDController.setTolerance(Constants.DrivetrainConstants.HeadingPID.headingPIDTolerance);
    m_gyro = new AHRS(SerialPort.Port.kMXP);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
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

  public void setDriveSetpoints(double left, double right){
    m_leftMotorLead.set(TalonFXControlMode.Position, left * Constants.DrivetrainConstants.ticksPerInch);
    m_rightMotorLead.set(TalonFXControlMode.Position, right * Constants.DrivetrainConstants.ticksPerInch);
    m_leftMotors[1].set(TalonFXControlMode.Position, left * Constants.DrivetrainConstants.ticksPerInch);
    m_rightMotors[1].set(TalonFXControlMode.Position, right * Constants.DrivetrainConstants.ticksPerInch);
  }

  public double getDriveSetpoints(){
    return m_leftMotorLead.getClosedLoopTarget();
  }

  public boolean atDriveSetpoints(){
    return (
      (m_leftMotorLead.getClosedLoopError() < Constants.DrivetrainConstants.DrivePID.drivePIDTolerance * Constants.DrivetrainConstants.ticksPerInch &&
      m_leftMotorLead.getClosedLoopError() > -Constants.DrivetrainConstants.DrivePID.drivePIDTolerance * Constants.DrivetrainConstants.ticksPerInch) &&
      (m_rightMotorLead.getClosedLoopError() < Constants.DrivetrainConstants.DrivePID.drivePIDTolerance * Constants.DrivetrainConstants.ticksPerInch &&
      m_rightMotorLead.getClosedLoopError() > -Constants.DrivetrainConstants.DrivePID.drivePIDTolerance * Constants.DrivetrainConstants.ticksPerInch)
    );
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
    return headingPIDController.calculate(value);
  }

  public double headingOutput(double value, double setPoint){
    return headingPIDController.calculate(value, setPoint);
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
