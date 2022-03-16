// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hanger.hangerStates;

public class HandleHanger extends CommandBase {
  /** Creates a new HandleHanger. */

  private final Hanger m_hanger;

  private boolean sensorState;

  public HandleHanger(Hanger hanger) {

    m_hanger = hanger;

    addRequirements(m_hanger);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hanger.stopHanger();
    m_hanger.setHangerState(m_hanger.getSelectedHangerState());
    sensorState = m_hanger.getMagnetDigitalInput();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    if(RobotContainer.getRaiseHangerButton()){  
      m_hanger.raiseHanger();
    }else if(RobotContainer.getLowerHangerButton()){
      m_hanger.lowerHanger();
    }else {
      m_hanger.stopHanger();
    }*/
    SmartDashboard.putBoolean("lower hanger", RobotContainer.getLowerHangerButton());
    SmartDashboard.putBoolean("raise hanger", RobotContainer.getRaiseHangerButton());
    


    if(sensorState && m_hanger.getCurrentHangerState() == hangerStates.RAISED && RobotContainer.getLowerHangerButton()){
      m_hanger.setHangerState(hangerStates.LOWERING);
    } else if(sensorState && m_hanger.getCurrentHangerState() == hangerStates.LOWERED && RobotContainer.getRaiseHangerButton()){
      m_hanger.setHangerState(hangerStates.RAISING);
    }

   if(m_hanger.getCurrentHangerState() == hangerStates.RAISING){
     if(sensorState && !m_hanger.getMagnetDigitalInput()){
       sensorState = false;
     } else if(!sensorState && m_hanger.getMagnetDigitalInput()){
       sensorState = true;
       m_hanger.setHangerState(hangerStates.RAISED);
     } else {
       m_hanger.raiseHanger();
     }
   }else if(m_hanger.getCurrentHangerState() == hangerStates.LOWERING){
     if(sensorState && !m_hanger.getMagnetDigitalInput()){
       sensorState = false;
     } else if(!sensorState && m_hanger.getMagnetDigitalInput()){
       sensorState = true;
       m_hanger.setHangerState(hangerStates.LOWERED);
     } else {
       m_hanger.lowerHanger();
     }
   } else {
     m_hanger.stopHanger();
   }

    // if(m_hanger.getCurrentHangerState() == hangerStates.LOWERED && !m_hanger.getMagnetDigitalInput()){
    //   m_hanger.raiseHanger();
    // }

    if(m_hanger.getCurrentHangerState() == hangerStates.RAISED && !m_hanger.getMagnetDigitalInput()){
      m_hanger.lowerHanger();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
