// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.SwerveDrive;

public class DriveToPosition extends CommandBase { 
  SwerveDrive m_drive;
  double m_x, m_y, m_thetaDegrees, m_maxMps, m_translationTolerance;
  Pose2d m_targetPose;

  /** Creates a new DriveToPosition. */
  public DriveToPosition(SwerveDrive drive, double x, double y, double thetaDegrees, double maxMps, double translationTolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive;
    m_x = x;
    m_y = y; // negative is workaround for coordinate issue
    m_thetaDegrees = thetaDegrees;
    m_maxMps = maxMps;
    m_targetPose = new Pose2d(m_x, m_y, Rotation2d.fromDegrees(m_thetaDegrees));
    m_translationTolerance = translationTolerance;
  }
  
  public static DriveToPosition CreateAutoCommand(ParsedCommand pc, SwerveDrive swerveDrive) {
    var x = AutoUtil.parseDouble(pc.getArgument("x"), 0.0);
    var y = AutoUtil.parseDouble(pc.getArgument("y"), 0.0);
    var angle = AutoUtil.parseDouble(pc.getArgument("angle"), 0.0);
    var maxMps = AutoUtil.parseDouble(pc.getArgument("maxMps"), Constants.MaxMPS);
    var translationTolerance = AutoUtil.parseDouble(pc.getArgument("translationTolerance"), Constants.DriveToPositionTolerance);
    return new DriveToPosition(swerveDrive, x, y, angle, maxMps, translationTolerance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setTargetPose(m_targetPose);
    m_drive.resetDriveToPosition();
    m_drive.setDriveTranslationTolerance(m_translationTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // In memory of the old code RIP 😥, Josh made do it
    m_drive.driveToPosition(m_maxMps);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.resetDriveToPosition();
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.isAtTargetPose();
  }
}
