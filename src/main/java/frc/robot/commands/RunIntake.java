// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;

public class RunIntake extends CommandBase {
  /** Creates a new RunIntake. */

  private final Intake m_intake;
  private final Uptake m_uptake;
  
  public RunIntake(Intake intake, Uptake uptake) {
    m_intake = intake;
    m_uptake = uptake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_uptake, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_uptake.isUptakeFull()){
      m_uptake.runUptake();
    }
    m_intake.runIntakeMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntakeMotor();
    m_uptake.stopUptake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
