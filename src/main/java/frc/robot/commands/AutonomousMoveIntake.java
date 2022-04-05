// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Intake.armStates;


public class AutonomousMoveIntake extends CommandBase {

  private final Intake m_intake;
  private final Uptake m_uptake;

  private boolean sensorState;
  private BooleanSupplier m_supplier;
  private int m_pos;

  private armStates m_armState;

  /** Creates a new LowerIntakeAuto. */
  public AutonomousMoveIntake(Intake intake, Uptake uptake, armStates armState, BooleanSupplier supplier, int position) {
    m_intake = intake;
    m_uptake = uptake;
    m_armState = armState;
    m_supplier = supplier;
    m_pos = position;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.stopIntakeArm();
    m_intake.setArmState(m_armState);
    m_intake.setArmRelPos(-m_pos);
    sensorState = m_intake.getMagnetDigitalInput();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_uptake.runUptake();
    m_intake.runIntakeMotor();/*
   if(m_intake.getCurrentArmState() == armStates.RAISING){
     if(sensorState && !m_intake.getMagnetDigitalInput()){
       sensorState = false;
     } else if(!sensorState && m_intake.getMagnetDigitalInput()){
       sensorState = true;
       m_intake.setArmState(armStates.RAISED);
     } else {
       m_intake.raiseIntakeArm();
     }
   }else if(m_intake.getCurrentArmState() == armStates.LOWERING){
     if(sensorState && !m_intake.getMagnetDigitalInput()){
       sensorState = false;
     } else if(!sensorState && m_intake.getMagnetDigitalInput()){
       sensorState = true;
       m_intake.setArmState(armStates.LOWERED);
     } else {
       m_intake.lowerIntakeArm();
     }
   } else {
     m_intake.stopIntakeArm();
   }*/
   if( m_intake.isArmBelowLowered()){
    m_intake.setIntakeArmPower(-.4); 

  if(m_intake.isArmBelowLoweredSlow()){
    m_intake.raiseIntakeArm();

    }
  } else if (m_intake.isArmAboveLowered()){
    m_intake.setIntakeArmPower(.6); 

  if(m_intake.isArmAboveLoweredSlow()){
    m_intake.lowerIntakeArm();

    }
  } else {
    m_intake.stopIntakeArm();
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntakeArm();
    //m_uptake.stopUptake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return m_supplier.getAsBoolean();//(!m_intake.isArmBelowLowered() && !m_intake.isArmAboveLowered());//(m_intake.getCurrentArmState() == armStates.LOWERED) || (m_intake.getCurrentArmState() == armStates.RAISED);
  }
}

