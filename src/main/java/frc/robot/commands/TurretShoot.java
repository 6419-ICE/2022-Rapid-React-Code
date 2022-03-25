// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.shooterStates;

public class TurretShoot extends CommandBase {
  /** Creates a new TurretShoot. */
  private final Uptake m_uptake;
  private final Shooter m_shooter;
  private double time;

  private shooterStates m_shooterState;

  public TurretShoot(Uptake uptake, Shooter shooter, shooterStates shooterState) {

    m_uptake = uptake;
    m_shooter = shooter;
    m_shooterState = shooterState;
    addRequirements(m_uptake, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(m_shooterState == shooterStates.HIGH){
      m_shooter.spoolUpHigh();
      if(m_shooter.timerReady(time, 1.5)){
        m_uptake.runUptake();
        if(m_uptake.isCargoPresent()){
          m_uptake.runLoader();
        }
      }
    }

    if(m_shooterState == shooterStates.LOW){
      m_shooter.spoolUpLow();
      if(m_shooter.timerReady(time, 1.5)){
          m_uptake.runUptake();
          if (m_uptake.isCargoPresent()){
            m_uptake.runLoader();
          } 
      } 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.spoolDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
