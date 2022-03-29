// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.pid.PIDTuner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveDrive;

public class DriverRelativeDriveWithAimSimple extends BaseRelativeDrive {
  private final static PIDController pid = new PIDController(0.04, 0, 0);
  public final static PIDTuner pidTuner = new PIDTuner("AimSimple", Robot.EnablePIDTuning, pid);
  private Camera m_camera;

  public DriverRelativeDriveWithAimSimple(SwerveDrive drive, Gamepad controller, Camera camera) {
    super(drive, controller);
    addRequirements(camera);
    m_camera = camera;
  }

  @Override
  public void initialize() {
      super.initialize();
      m_camera.setPipeline(Camera.ComputerVision);
  }

  @Override
  public void moveRobot(double sidewaySpeed, double forwardSpeed, double omegaSpeed) {
    if(m_camera.hasTarget()) {
      omegaSpeed = MathUtil.clamp(pid.calculate(m_camera.getXAngle()), -Constants.MaxORPS, Constants.MaxORPS); // TODO: Might need to invert
    } 
    m_drive.moveDriverRelative(-sidewaySpeed, forwardSpeed, omegaSpeed);
  }
}
