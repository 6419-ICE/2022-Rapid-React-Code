// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Uptake;

public class HandleUptake extends CommandBase {
  private final Uptake m_uptake;

  /** Creates a new HandleUptake. */
  public HandleUptake(Uptake uptake) {
    m_uptake = uptake;
    addRequirements(m_uptake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_uptake.stopLoader();
    m_uptake.stopUptake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.getReverseIntakeButton()){
      m_uptake.reverseUptake();
    }else if(RobotContainer.getRunIntakeButton() && !m_uptake.isCargoPresent()){
      m_uptake.runUptake();
    } else {
      m_uptake.stopUptake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_uptake.stopLoader();
    m_uptake.stopUptake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
