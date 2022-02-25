// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hanger;

public class HandleHanger extends CommandBase {
  /** Creates a new HandleHanger. */

  private final Hanger m_hanger;

  public HandleHanger(Hanger hanger) {

    m_hanger = hanger;

    addRequirements(m_hanger);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.getRaiseHangerButton()){  
      m_hanger.raiseHanger();
    }else if(RobotContainer.getLowerHangerButton()){
      m_hanger.lowerHanger();
    }else {
      m_hanger.stopHanger();
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
