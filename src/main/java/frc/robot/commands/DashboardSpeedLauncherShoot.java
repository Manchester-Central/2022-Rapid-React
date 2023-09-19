// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Feeder.FeederMode;

public class DashboardSpeedLauncherShoot extends BaseLauncherShoot {

  /** Creates a new LauncherShoot. */
  public DashboardSpeedLauncherShoot(Launcher launcher, Feeder feeder) {
    super(launcher, feeder, FeederMode.LAUNCH_CAMERA);
    SmartDashboard.putNumber("DashboardLauncherSpeed", 9500);
    SmartDashboard.putNumber("DashboardLauncherTolerance", Constants.DefaultLauncherTolerance);
    SmartDashboard.putNumber("DashboardFeederSpeed", Constants.DefaultFeederLaunchSpeed);
  }

  @Override
  protected double getTargetSpeed() {
    var launcherTolerance = SmartDashboard.getNumber("DashboardLauncherTolerance", 0);
    m_launcher.setLauncherTolerance(launcherTolerance);
    var feederSpeed = SmartDashboard.getNumber("DashboardFeederSpeed", 0);
    FeederDefault.DefaultCameraLaunchSpeed = feederSpeed;
    return SmartDashboard.getNumber("DashboardLauncherSpeed", 0);
  }
}
