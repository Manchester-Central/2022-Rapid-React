// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveToPosition extends CommandBase {
  private final double k_maxSpeedMps = 4;
  SwerveDrive m_drive;
  double m_x, m_y;
  Pose2d m_targetPose;
  /** Creates a new DriveToPosition. */
  public DriveToPosition(SwerveDrive drive, double x, double y) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive;
    m_x = x;
    m_y = y;
    m_targetPose = new Pose2d(x, y, new Rotation2d(0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var difference = getDistanceFromTarget();
    SmartDashboard.putString("difference", difference.toString());
    double diffX = difference.getX();
    double diffY = difference.getY();
    double distance = Math.sqrt(Math.pow(diffX, 2) + Math.pow(diffY, 2));
    double speedX = k_maxSpeedMps * diffX / distance;
    double speedY = k_maxSpeedMps * diffY / distance;
    m_drive.moveFieldRelative(speedX, speedY, 0);
  }

  private Translation2d getDistanceFromTarget() {
    var currentPose = m_drive.getPose();
    var difference = m_targetPose.getTranslation().minus(currentPose.getTranslation());
    return difference;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var difference = getDistanceFromTarget();
    return Math.abs(difference.getX()) < 0.01 && Math.abs(difference.getY()) < 0.01;
  }
}
