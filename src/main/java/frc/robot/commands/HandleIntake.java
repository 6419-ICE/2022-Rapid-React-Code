// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Intake.armStates;
import frc.robot.subsystems.Intake.armModes;

public class HandleIntake extends CommandBase {

  private final Intake m_intake;

  private boolean sensorState;
  private boolean isMoving;
  private int setPosition;

  /** Creates a new HandleIntake. */
  public HandleIntake(Intake intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.stopIntakeArm();
    m_intake.stopIntakeMotor();
    if(m_intake.getCurrentArmState() == null){
      m_intake.setArmState(m_intake.getSelectedArmState());
    }
    setPosition = m_intake.getArmAbsPos();
    sensorState = m_intake.getMagnetDigitalInput();
    isMoving = false;

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
    
    if(m_intake.getSelectedArmMode() == armModes.MANUAL){
      if(RobotContainer.getLowerIntakeButton()){
        m_intake.lowerIntakeArm();
        isMoving = true;

      } else if(RobotContainer.getRaiseIntakeButton()){
        m_intake.raiseIntakeArm();
        isMoving = true;

      } else {
        if(isMoving){
          isMoving = false;
          setPosition = m_intake.getArmAbsPos();
        }
        m_intake.stopIntakeArm();
      }
      
      if(RobotContainer.getLockArmButton()){
        m_intake.resetArmAbsPos();
      }
      if(!isMoving){
      if(Math.abs(m_intake.getArmAbsPos())-Math.abs(setPosition) < -500){
        m_intake.lowerIntakeArm();


      }else if(Math.abs(setPosition)-Math.abs(m_intake.getArmAbsPos()) > 500){
        m_intake.raiseIntakeArm();



      }else{
        m_intake.stopIntakeArm();
      }
    } 
      
    } else if(m_intake.getSelectedArmMode() == armModes.MAGNETIC){
      if((sensorState && m_intake.getCurrentArmState() == armStates.RAISED && RobotContainer.getLowerIntakeButton())){
        m_intake.setArmState(armStates.LOWERING);
      } else if(sensorState && m_intake.getCurrentArmState() == armStates.LOWERED && RobotContainer.getRaiseIntakeButton()){
        m_intake.setArmState(armStates.RAISING);
      }

      if(m_intake.getCurrentArmState() == armStates.RAISING){
        if(sensorState && !m_intake.getMagnetDigitalInput()){
          sensorState = false;
        } else if(!sensorState && m_intake.getMagnetDigitalInput()){
          sensorState = true;
          m_intake.setArmState(armStates.RAISED);
        } else {
          m_intake.raiseIntakeArm();
        }
      } else if(m_intake.getCurrentArmState() == armStates.LOWERING){
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
      }

      if(m_intake.getCurrentArmState() == armStates.LOWERED && !m_intake.getMagnetDigitalInput()){
        m_intake.raiseIntakeArm();
      }

      if(m_intake.getCurrentArmState() == armStates.RAISED && !m_intake.getMagnetDigitalInput()){
        m_intake.lowerIntakeArm();
      }
    }else if(m_intake.getCurrentArmMode() == armModes.ENCODER){
      /*if(RobotContainer.getRaiseIntakeButton()){
        m_intake.setGoal(Constants.IntakeConstants.encoderRaisedPosition);
      } else if (RobotContainer.getLowerIntakeButton()){
        m_intake.setGoal(Constants.IntakeConstants.encoderLoweredPosition);
      }*/
    }
    SmartDashboard.putNumber("setpoint", setPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntakeArm();
    m_intake.stopIntakeMotor();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private String returnArmState(){
    return m_intake.getCurrentArmState().toString();
  }
}
