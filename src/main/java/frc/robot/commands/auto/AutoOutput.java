// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder.FeederMode;

public class AutoOutput extends CommandBase {
  public static boolean isIntakeReversed = false;
  private Feeder m_feeder;
  private Intake m_intake;
  /** Creates a new Output. */
  public AutoOutput(Feeder feeder, Intake intake) {
    m_feeder = feeder;
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {isIntakeReversed = true;}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feeder.setFeederMode(FeederMode.OUTPUT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isIntakeReversed = false;
    m_feeder.setFeederMode(FeederMode.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
