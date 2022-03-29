// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AngleUtil;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveModulePosition;

public class AimToGoal extends CommandBase {
  private SwerveDrive m_swerveDrive;
  private Camera m_camera;
  private final Translation2d k_goalLocation = new Translation2d(Constants.GoalX, Constants.GoalY);

  /** Creates a new AimToGoal. */
  public AimToGoal(SwerveDrive swerveDrive, Camera camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveDrive = swerveDrive;
    m_camera = camera;
    addRequirements(swerveDrive, camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_camera.setPipeline(Camera.ComputerVision);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var OmegaSpeed = 0.0;
    var currentPose = m_swerveDrive.getPose();
    var currentRotation = m_swerveDrive.getRotation();
    var rotation = AngleUtil.GetEstimatedAngleToGoal(m_camera, currentPose, currentRotation);
    m_swerveDrive.setTargetAngle(rotation);
    OmegaSpeed = m_swerveDrive.getTargetOmega();
    SmartDashboard.putNumber("aim/omega", OmegaSpeed);
    m_swerveDrive.moveRobotRelative(0, 0, OmegaSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
    m_camera.setPipeline(Camera.HumanVision);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return m_camera.hasTarget() && Math.abs(m_camera.getXAngle()) < 0.5;
  }
}
