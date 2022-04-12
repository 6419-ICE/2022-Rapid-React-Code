// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.shooterStates;

public class AutonomousShoot extends CommandBase {
  /** Creates a new AutonomousShoot. */
  private final Shooter m_shooter;
  private final Uptake m_uptake;

  private Integer m_maxMs;

  private double time;

  private boolean firstBall;
  private boolean secondBall;
  private boolean isDone;

  private shooterStates m_shooterState;

  public AutonomousShoot( Uptake uptake, Shooter shooter, shooterStates shooterState, Integer maxMs) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_uptake = uptake;
    m_shooterState = shooterState;
    m_maxMs = maxMs;
    addRequirements(m_shooter, m_uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
    firstBall = true;
    secondBall = false;
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putBoolean("isDone", isDone);
    // SmartDashboard.putBoolean("firstBall", firstBall);
    // SmartDashboard.putBoolean("secondBall", secondBall);
    SmartDashboard.putNumber("Shooter Speed", m_shooter.getShooterSpeed());
    Integer timeOut = 0;
    
    if(m_shooterState == shooterStates.HIGH){
      m_shooter.spoolUpHigh();
      if(!m_shooter.isShooterReadyHigh() && timeOut < m_maxMs){
        Timer.delay(0.001);
      } if(timeOut < m_maxMs && m_shooter.isShooterReadyHigh()){
        m_uptake.runUptake();
        if(m_uptake.isCargoPresentTop()){
          m_uptake.runLoader();
        }
      }
    } 
    if (m_shooterState == shooterStates.LOW ){
      m_shooter.spoolUpLow();
      if(m_shooter.timerReady(time, Constants.ShooterConstants.shootTimer)){
        m_uptake.runUptake();
        if(m_uptake.isCargoPresentTop()){
          m_uptake.runUptake();
        }
      }
    }
    /*if(firstBall){
      if(!m_uptake.isCargoPresentTop()){
        firstBall = false;
      }
    } else {
      if(!secondBall && m_uptake.isCargoPresentTop()){
        secondBall = true;
      } else if(secondBall && !m_uptake.isCargoPresentTop()){
        isDone = true;
      }
    }*/
    if(!m_uptake.isCargoPresentTop() && secondBall){
      isDone = true;
    }
    if(!m_uptake.isCargoPresentTop()){
      m_uptake.runUptake();
      firstBall = false;
    }
    if(m_uptake.isCargoPresentTop() && !firstBall){
      secondBall = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      SmartDashboard.putBoolean("Is Interrupted", interrupted);
    }
    //Timer.delay(0.5);
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
