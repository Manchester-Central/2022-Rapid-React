// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Feeder.FeederMode;

public abstract class BaseLauncherShoot extends CommandBase {
  protected Launcher m_launcher;
  protected Feeder m_feeder;
  protected final double DoNotLaunchSpeed = -1.0;

  /** Creates a new LauncherShoot. */
  public BaseLauncherShoot(Launcher launcher, Feeder feeder) {
    m_launcher = launcher;
    m_feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcher, feeder); // remove feeder as requirement. See PR #75
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public final void execute() {
    var speed = getTargetSpeed();
    if (getTargetSpeed() != DoNotLaunchSpeed) {
      m_launcher.SetTargetRPM(speed);

      if (m_launcher.isAtTargetSpeed(speed)) {
        m_feeder.setFeederMode(FeederMode.LAUNCH);
      } else {
        m_feeder.setFeederMode(FeederMode.DEFAULT);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public final void end(boolean interrupted) {
    m_launcher.coast();
    m_feeder.setFeederMode(FeederMode.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  protected abstract double getTargetSpeed();
}
