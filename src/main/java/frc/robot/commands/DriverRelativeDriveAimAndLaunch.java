// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.AngleUtil;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.FlywheelTable;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Feeder.FeederMode;

public class DriverRelativeDriveAimAndLaunch extends BaseRelativeDrive {
  private Camera m_camera;
  private FlywheelTable m_flywheelTable;
  private Launcher m_launcher;
  private Feeder m_feeder;

  public DriverRelativeDriveAimAndLaunch(SwerveDrive drive, Gamepad controller, Camera camera, Launcher launcher,
      FlywheelTable flywheelTable, Feeder feeder) {
    super(drive, controller, true);
    addRequirements(camera, launcher);
    m_camera = camera;
    m_launcher = launcher;
    m_flywheelTable = flywheelTable;
    m_feeder = feeder;
  }

  @Override
  public void initialize() {
    super.initialize();
    FeederDefault.DefaultCameraLaunchSpeed = Constants.DefaultFeederLaunchSpeed;
    m_camera.setPipeline(Camera.ComputerVision);
    m_drive.deleteDriveToPositionError();
    m_feeder.setFeederMode(FeederMode.DEFAULT);
    var speed = m_flywheelTable.getIdealTarget(-25).getSpeed();
    m_launcher.SetTargetRPM(speed);
  }

  @Override
  public void moveRobot(double sidewaySpeed, double forwardSpeed, double omegaSpeed) {
    if (m_camera.hasTarget()) {
      var currentPose = m_drive.getPose();
      var currentRotation = m_drive.getRotation();
      var rotation = AngleUtil.GetEstimatedAngleToGoal(m_camera, currentPose, currentRotation);
      m_drive.setTargetAngle(rotation);
      omegaSpeed = m_drive.getTargetOmega();

      var speed = getTargetSpeed();
      m_launcher.SetTargetRPM(speed);

      if (m_launcher.isAtTargetSpeed(speed) && m_drive.isAtTargetAngle()) {
        m_feeder.setFeederMode(FeederMode.LAUNCH_CAMERA);
      } else {
        m_feeder.setFeederMode(FeederMode.DEFAULT);
      }
    }
    m_drive.moveDriverRelative(-sidewaySpeed, forwardSpeed, omegaSpeed);
  }

  protected double getTargetSpeed() {
    if (m_camera.hasTarget()) {
      var distance = m_camera.getYAngle();
      var target = m_flywheelTable.getIdealTarget(distance);
      var speed = target.getSpeed();

      if (DriverStation.isAutonomous()) {
        speed += 150;
      }

      if (target.getHoodUp()) {
        m_launcher.MoveHoodUp();
      } else {
        m_launcher.MoveHoodDown();
      }

      m_launcher.setLauncherTolerance(target.getLauncherTolerance());
      FeederDefault.DefaultCameraLaunchSpeed = target.getFeederSpeed();

      return speed;
    }
    return 0;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_launcher.coast();
    m_feeder.setFeederMode(FeederMode.DEFAULT);
  }
}
