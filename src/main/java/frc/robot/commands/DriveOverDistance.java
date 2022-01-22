// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveOverDistance extends CommandBase {
  Pose2d m_startingPosition;
  SwerveDrive m_drive;
  float m_x, m_y, m_theta;
  double m_distanceM;
  /** Creates a new DriveOverDistance. */
  public DriveOverDistance(SwerveDrive drive, float x, float y, float theta, double distanceM) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive;
    m_x = x;
    m_y = y;
    m_theta = theta;
    m_distanceM = distanceM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startingPosition = m_drive.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.moveFieldRelative(m_x, m_y, m_theta);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceTraveled = m_drive.getPose().getTranslation().getDistance(m_startingPosition.getTranslation());
    return distanceTraveled > m_distanceM;
  }
}
