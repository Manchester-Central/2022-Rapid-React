// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.auto.ParsedCommand;

import frc.robot.Constants;
import frc.robot.commands.auto.AutoUtil;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Feeder.FeederMode;

public class SetSpeedLauncherShoot extends BaseLauncherShoot {
  private double m_manualSpeed;
  private boolean m_setHoodUp;
  private FeederMode m_feederMode;
  private double m_launcherTolerance;

  /** Creates a new LauncherShoot. */
  public SetSpeedLauncherShoot(Launcher launcher, Feeder feeder, double manualSpeed, FeederMode feederMode, double launcherTolerance) {
    super(launcher, feeder, feederMode);
    m_manualSpeed = manualSpeed;
    m_launcherTolerance = launcherTolerance;
    m_setHoodUp = feederMode == FeederMode.LAUNCH_HIGH_BUMPER;
    m_feederMode = feederMode;
  }

  @Override
  public void initialize() {
    super.initialize();
    if(m_setHoodUp) {
      m_launcher.MoveHoodUp();
    }
    else {
      m_launcher.MoveHoodDown();
    }
    m_launcher.setLauncherTolerance(m_launcherTolerance);
  }
  
  public static SetSpeedLauncherShoot CreateAutoCommand(ParsedCommand pc, Launcher launcher, Feeder feeder) {
    var speed = AutoUtil.parseDouble(pc.getArgument("speed"), Constants.DefaultLauncherLowSpeed);
    return new SetSpeedLauncherShoot(launcher, feeder, speed, FeederMode.LAUNCH_CAMERA, Constants.DefaultLauncherTolerance);
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
