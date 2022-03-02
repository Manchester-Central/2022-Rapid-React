// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.SwerveDrive;

public class DriveToPosition extends CommandBase {
  private final double k_maxSpeedMps = 1;
  private final double k_slowDownDistanceM = 1;
  private final double k_maxRotationChange = 4;
  private final double k_slowDownAngleDegrees = 30;
  SwerveDrive m_drive;
  double m_x, m_y, m_thetaDegrees;
  Pose2d m_targetPose;

  /** Creates a new DriveToPosition. */
  public DriveToPosition(SwerveDrive drive, double x, double y, double thetaDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive;
    m_x = x;
    m_y = y;
    m_thetaDegrees = thetaDegrees;
    m_targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(thetaDegrees));
  }
  
  public static DriveToPosition CreateAutoCommand(ParsedCommand pc, SwerveDrive swerveDrive) {
    var x = AutoUtil.parseDouble(pc.getArgument("x"), 0.0);
    var y = AutoUtil.parseDouble(pc.getArgument("y"), 0.0);
    var angle = AutoUtil.parseDouble(pc.getArgument("angle"), 0.0);
    return new DriveToPosition(swerveDrive, x, y, angle);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var difference = getDistanceFromTarget();
    double diffX = difference.getX();
    double diffY = difference.getY();
    double distance = Math.sqrt(Math.pow(diffX, 2) + Math.pow(diffY, 2));
    double speedX = k_maxSpeedMps * diffX / distance;
    double speedY = k_maxSpeedMps * diffY / distance;
    if (distance < k_slowDownDistanceM) {
      double slowDownRatio = distance / k_slowDownDistanceM;
      speedX *= slowDownRatio;
      speedY *= slowDownRatio;
    }
    var rotationDifference = getRotationFromTarget();
    var rotationDifferenceDegrees = rotationDifference.getDegrees();
    var rotationChangeSpeed = rotationDifferenceDegrees < 0 ? -k_maxRotationChange : k_maxRotationChange;
    if (Math.abs(rotationDifferenceDegrees) < k_slowDownAngleDegrees) {
      double slowDownRatio = Math.abs(rotationDifferenceDegrees) / k_slowDownAngleDegrees;
      rotationChangeSpeed *= slowDownRatio;
    }
    m_drive.moveFieldRelative(speedX, speedY, rotationChangeSpeed);
  }

  private Translation2d getDistanceFromTarget() {
    var currentPose = m_drive.getPose();
    var difference = m_targetPose.getTranslation().minus(currentPose.getTranslation());
    return difference;
  }

  private Rotation2d getRotationFromTarget() {
    var currentPose = m_drive.getPose();
    var currentRotation = currentPose.getRotation();
    var targetRotation = m_targetPose.getRotation();
    var difference = targetRotation.minus(currentRotation);
    return difference;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var difference = getDistanceFromTarget();
    var rotationDifference = getRotationFromTarget();
    var isXGood = Math.abs(difference.getX()) < 0.01;
    var isYGood = Math.abs(difference.getY()) < 0.01;
    var isRotationGood = Math.abs(rotationDifference.getDegrees()) < 0.1;
    return isXGood && isYGood && isRotationGood;
  }
}
