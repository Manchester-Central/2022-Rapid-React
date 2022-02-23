// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class FeederDefault extends CommandBase {
  private Feeder m_feeder;
  /** Creates a new FeederDefault. */
  public FeederDefault(Feeder feeder) {
    m_feeder = feeder;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_feeder.IsBallAtTopFeeder() && m_feeder.IsBallAtMiddleFeeder()){
      m_feeder.Stop();
    }
    else if(m_feeder.IsBallAtTopFeeder()){
      m_feeder.Bottom();
    }
    else {
      m_feeder.Both();
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.Stop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
