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
import frc.robot.subsystems.Intake.armStates;

public class HandleIntake extends CommandBase {

  private final Intake m_intake;
  private armStates armState;
  private boolean sensorState;

  /** Creates a new HandleIntake. */
  public HandleIntake(Intake intake) {
    m_intake = intake;
    armState = armStates.RAISED;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.stopIntakeArm();
    m_intake.stopIntakeMotor();
    sensorState = m_intake.getMagnetDigitalInput();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.getReverseIntakeButton()){
      m_intake.reverseIntakeMotor();
    }else if(RobotContainer.getRunIntakeButton()){
      m_intake.runIntakeMotor();
    } else {
      m_intake.stopIntakeMotor();
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
    if(sensorState && armState == armStates.RAISED && RobotContainer.getLowerIntakeButton()){
      armState = armStates.LOWERING;
    } else if(sensorState && armState == armStates.LOWERED && RobotContainer.getRaiseIntakeButton()){
      armState = armStates.RAISING;
    }

    if(armState == armStates.RAISING){
      if(sensorState && !m_intake.getMagnetDigitalInput()){
        sensorState = false;
      } else if(!sensorState && m_intake.getMagnetDigitalInput()){
        sensorState = true;
        armState = armState.RAISED;
      } else {
        m_intake.raiseIntakeArm();
      }
    }

    if(armState == armStates.LOWERING){
      if(sensorState && !m_intake.getMagnetDigitalInput()){
        sensorState = false;
      } else if(!sensorState && m_intake.getMagnetDigitalInput()){
        sensorState = true;
        armState = armState.LOWERED;
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
    return armState.toString();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("Arm State", this::returnArmState, null);
  }
}
