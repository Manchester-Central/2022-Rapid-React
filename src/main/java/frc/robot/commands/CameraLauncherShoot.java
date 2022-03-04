// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.FlywheelTable;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Feeder.FeederMode;

public class CameraLauncherShoot extends BaseLauncherShoot {
  private Camera m_camera;
  private FlywheelTable m_flyWheelTable;

  /** Creates a new LauncherShoot. */
  public CameraLauncherShoot(Launcher launcher, Camera camera, Feeder feeder, FlywheelTable flyWheel) {
    super(launcher, feeder);
    m_camera = camera;
    m_flyWheelTable = flyWheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_camera.setPipeline(0);
    var speed = m_flyWheelTable.getIdealTarget(-25).getSpeed();
    m_launcher.SetTargetRPM(speed);
    m_feeder.setFeederMode(FeederMode.DEFAULT);
  }

  @Override
  protected double getTargetSpeed() {
    if (m_camera.hasTarget()) {
      var distance = m_camera.getDistance();
      return m_flyWheelTable.getIdealTarget(distance).getSpeed();
    }
    return DoNotLaunchSpeed;
  }
}
