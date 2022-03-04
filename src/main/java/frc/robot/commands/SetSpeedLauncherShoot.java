// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import frc.robot.Constants;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Launcher;

public class SetSpeedLauncherShoot extends BaseLauncherShoot {
  private double m_manualSpeed;

  /** Creates a new LauncherShoot. */
  public SetSpeedLauncherShoot(Launcher launcher, Feeder feeder, double manualSpeed) {
    super(launcher, feeder);
    m_manualSpeed = manualSpeed;
  }
  
  public static SetSpeedLauncherShoot CreateAutoCommand(ParsedCommand pc, Launcher launcher, Feeder feeder) {
    var speed = AutoUtil.parseDouble(pc.getArgument("speed"), Constants.DefaultLauncherLowSpeed);
    return new SetSpeedLauncherShoot(launcher, feeder, speed);
  }

  @Override
  protected double getTargetSpeed() {
    return m_manualSpeed;
  }
}
