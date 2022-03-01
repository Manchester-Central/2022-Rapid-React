// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveDrive;

public class AimToGoal extends CommandBase {
  private SwerveDrive m_swerveDrive;
  private Camera m_camera;
  private final Translation2d k_goalLocation = new Translation2d(0, 0); // TODO: Get real values

  /** Creates a new AimToGoal. */
  public AimToGoal(SwerveDrive swerveDrive, Camera camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveDrive = swerveDrive;
    m_camera = camera;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_camera.hasTarget()) {
      var thetaSpeed = m_camera.getXAngle();
      thetaSpeed = MathUtil.clamp(thetaSpeed, -Constants.MaxORPS * 0.5, Constants.MaxORPS * 0.5);
      m_swerveDrive.moveRobotRelative(0, 0, thetaSpeed);
    } else {
      var currentLocation = m_swerveDrive.getPose().getTranslation();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_camera.hasTarget() && Math.abs(m_camera.getXAngle()) < 0.5;
  }
}
