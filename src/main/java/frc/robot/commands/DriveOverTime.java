// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveOverTime extends CommandBase {
  long m_startTimeMs;
  SwerveDrive m_drive;
  float m_x, m_y, m_theta;
  int m_timeMs;

  /** Creates a new DriveOverTime. */
  public DriveOverTime(SwerveDrive drive, float x, float y, float theta, int timeMs) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive;
    m_x = x;
    m_y = y;
    m_theta = theta;
    m_timeMs = timeMs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTimeMs = RobotController.getFPGATime() / 1000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.moveFieldRelative(m_x, m_y, m_theta);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    long currentTimeMs = RobotController.getFPGATime() / 1000;
    return currentTimeMs > m_startTimeMs + m_timeMs;
  }
}
