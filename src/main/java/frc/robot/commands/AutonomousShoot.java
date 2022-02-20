// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.shooterStates;

public class AutonomousShoot extends CommandBase {
  /** Creates a new AutonomousShoot. */
  private final Uptake m_uptake;
  private final Shooter m_shooter;

  private Integer m_maxMs;

  private boolean secondBall;
  private boolean isDone;

  private shooterStates m_shooterState;

  public AutonomousShoot(Uptake uptake, Shooter shooter, shooterStates shooterState, Integer maxMs) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_uptake = uptake;
    m_shooter = shooter;
    m_shooterState = shooterState;
    m_maxMs = maxMs;
    addRequirements(m_uptake, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    secondBall = false;
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Integer timeOut = 0;
    if(m_shooterState == shooterStates.HIGH){
      m_shooter.spoolUpHigh();
      while(!m_shooter.isShooterReadyHigh() && timeOut < m_maxMs){
        Timer.delay(0.001);
        timeOut++;
      }
      if(timeOut < m_maxMs){
        m_uptake.runLoader();
        while(m_uptake.isCargoPresent()){
          m_uptake.setUptakePower(.6);
        }
      }
    } else{
      m_shooter.spoolUpLow();
      while(!m_shooter.isShooterReadyLow()){
        Timer.delay(0.001);
        timeOut++;
      }
      SmartDashboard.putBoolean("Shooter Ready Low: ", m_shooter.isShooterReadyLow());
      // SmartDashboard.putNumber("Timeout Value", timeOut);
      SmartDashboard.putNumber("Max Timeout Value", m_maxMs);
      SmartDashboard.putBoolean("Second Ball", secondBall);
      SmartDashboard.putBoolean("Is Done", isDone);
      // if(timeOut < maxMs){
      SmartDashboard.putBoolean("Cargo is Present", m_uptake.isCargoPresent());
      m_uptake.runLoader();
      while(m_uptake.isCargoPresent()){
        m_uptake.setUptakePower(.6);
      }
    }
    if(secondBall){
      isDone = true;
    }
    while(!m_uptake.isCargoPresent()){
      m_uptake.runUptake();
    }
    secondBall = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Timer.delay(0.5);
    m_uptake.stopLoader();
    m_uptake.stopUptake();
    m_shooter.spoolDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
