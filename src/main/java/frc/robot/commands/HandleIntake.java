// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Intake.armStates;

public class HandleIntake extends CommandBase {

  private final Intake m_intake;
  private final Uptake m_uptake;

  private boolean sensorState;

  /** Creates a new HandleIntake. */
  public HandleIntake(Intake intake, Uptake uptake) {
    m_intake = intake;
    m_uptake = uptake;
    addRequirements(m_intake, m_uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.stopIntakeArm();
    m_intake.stopIntakeMotor();
    m_uptake.stopLoader();
    m_uptake.stopUptake();
    sensorState = m_intake.getMagnetDigitalInput();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.getReverseIntakeButton()){
      m_intake.reverseIntakeMotor();
    }else if(RobotContainer.getRunIntakeButton()){
      m_intake.runIntakeMotor();
      if(!m_uptake.isCargoPresent()){
        m_uptake.runUptake();
      }
    } else {
      m_intake.stopIntakeMotor();
      m_uptake.stopUptake();
    }
/*
    if(RobotContainer.getLowerIntakeButton()){
      m_intake.lowerIntakeArm();
    } else if(RobotContainer.getRaiseIntakeButton()){
      m_intake.raiseIntakeArm();
    } else {
      m_intake.stopIntakeArm();
    }
*/
    // if(sensorState && m_intake.getCurrentArmState() == armStates.RAISED && RobotContainer.getLowerIntakeButton()){
    //   m_intake.setArmState(armStates.LOWERING);
    // } else if(sensorState && m_intake.getCurrentArmState() == armStates.LOWERED && RobotContainer.getRaiseIntakeButton()){
    //   m_intake.setArmState(armStates.RAISING);
    // }

    if(m_intake.getCurrentArmState() == armStates.RAISING){
      if(sensorState && !m_intake.getMagnetDigitalInput()){
        sensorState = false;
      } else if(!sensorState && m_intake.getMagnetDigitalInput()){
        sensorState = true;
        m_intake.setArmState(armStates.RAISED);
      } else {
        m_intake.raiseIntakeArm();
      }
    }

    if(m_intake.getCurrentArmState() == armStates.LOWERING){
      if(sensorState && !m_intake.getMagnetDigitalInput()){
        sensorState = false;
      } else if(!sensorState && m_intake.getMagnetDigitalInput()){
        sensorState = true;
        m_intake.setArmState(armStates.LOWERED);
      } else {
        m_intake.lowerIntakeArm();
      }
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

  private String returnArmState(){
    return m_intake.getCurrentArmState().toString();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("Arm State", this::returnArmState, null);
  }
}
