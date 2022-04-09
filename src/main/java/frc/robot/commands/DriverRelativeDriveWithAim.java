// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import frc.robot.subsystems.AngleUtil;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveDrive;

public class DriverRelativeDriveWithAim extends BaseRelativeDrive {
  private Camera m_camera;

  public DriverRelativeDriveWithAim(SwerveDrive drive, Gamepad controller, Camera camera) {
    super(drive, controller);
    addRequirements(camera);
    m_camera = camera;
  }

  @Override
  public void initialize() {
    super.initialize();
    m_camera.setPipeline(Camera.ComputerVision);
    m_drive.resetDriveToPosition();
  }

  @Override
  public void moveRobot(double sidewaySpeed, double forwardSpeed, double omegaSpeed) {
    if (m_camera.hasTarget()) {
      var currentPose = m_drive.getPose();
      var currentRotation = m_drive.getRotation();
      var rotation = AngleUtil.GetEstimatedAngleToGoal(m_camera, currentPose, currentRotation);
      m_drive.setTargetAngle(rotation);
      omegaSpeed = m_drive.getTargetOmega();
    }
    m_drive.moveDriverRelative(-sidewaySpeed, forwardSpeed, omegaSpeed);
  }
}
