// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;

public class TurretFire extends CommandBase {
  /** Creates a new TurretFire. */

  private final Shooter m_shooter;
  private final Uptake m_uptake;

  private BooleanSupplier m_condition;

  public TurretFire(Shooter shooter, Uptake uptake, BooleanSupplier condition) {
    m_shooter = shooter;
    m_uptake = uptake;
    m_condition = condition;
    addRequirements(shooter, uptake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public TurretFire(Shooter shooter, Uptake uptake) {
    m_shooter = shooter;
    m_uptake = uptake;
    addRequirements(shooter, uptake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_condition != null){
      if(m_condition.getAsBoolean()){
        m_uptake.runUptake();
        if (m_uptake.isCargoPresent()){
          m_uptake.runLoader();
        }else {
          m_uptake.stopLoader();
        }
      }else{
        m_uptake.stopLoader();
        m_uptake.stopUptake();
      }
    } else {
      m_uptake.runUptake();
      if (m_uptake.isCargoPresent()){
        m_uptake.runLoader();
      }else {
        m_uptake.stopLoader();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.spoolDown();
    m_uptake.stopLoader();
    m_uptake.stopUptake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
