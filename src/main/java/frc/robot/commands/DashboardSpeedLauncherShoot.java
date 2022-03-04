// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Launcher;

public class DashboardSpeedLauncherShoot extends BaseLauncherShoot {

  /** Creates a new LauncherShoot. */
  public DashboardSpeedLauncherShoot(Launcher launcher, Feeder feeder) {
    super(launcher, feeder);
    SmartDashboard.putNumber("DashboardLauncherSpeed", Constants.DefaultLauncherLowSpeed);
  }

  @Override
  protected double getTargetSpeed() {
    return SmartDashboard.getNumber("DashboardLauncherSpeed", 0);
  }
}
