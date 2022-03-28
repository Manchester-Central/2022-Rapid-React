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
  private boolean m_setHoodUp;

  /** Creates a new LauncherShoot. */
  public SetSpeedLauncherShoot(Launcher launcher, Feeder feeder, double manualSpeed, boolean setHoodUp) {
    super(launcher, feeder);
    m_manualSpeed = manualSpeed;
    m_setHoodUp = setHoodUp;
  }

  @Override
  public void initialize() {
    super.initialize();
    if(m_setHoodUp) {
      m_launcher.MoveHoodUp();
    }
  }
  
  public static SetSpeedLauncherShoot CreateAutoCommand(ParsedCommand pc, Launcher launcher, Feeder feeder) {
    var speed = AutoUtil.parseDouble(pc.getArgument("speed"), Constants.DefaultLauncherLowSpeed);
    return new SetSpeedLauncherShoot(launcher, feeder, speed, false);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if(m_setHoodUp) {
      m_launcher.MoveHoodDown();
    }
  }

  @Override
  protected double getTargetSpeed() {
    return m_manualSpeed;
  }
}
