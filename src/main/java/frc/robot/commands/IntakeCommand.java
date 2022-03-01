// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  private Feeder m_feeder;
  private Intake m_intake;

  /** Creates a new Output. */
  public IntakeCommand(Feeder feeder, Intake intake) {
    m_feeder = feeder;
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_feeder.IsBallAtTopFeeder() && m_feeder.IsBallAtMiddleFeeder()) {
      m_feeder.Stop();
      m_intake.ManualIntake(0);
    } else if (m_feeder.IsBallAtTopFeeder()) {
      m_intake.ManualIntake(1);
      m_feeder.ManualFeed(0, 0.5);
    } else {
      m_intake.ManualIntake(1);
      m_feeder.ManualFeed(0.15, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.Stop();
    m_intake.ManualIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
