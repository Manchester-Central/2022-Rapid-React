// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.FlywheelTable;
import frc.robot.subsystems.Launcher;

public class LauncherShoot extends CommandBase {
  private Launcher m_launcher;
  private Camera m_camera;
  private Feeder m_feeder;
  private FlywheelTable m_flyWheelTable;
  //private double m_targetSpeed;

  /** Creates a new LauncherShoot. */
  public LauncherShoot(Launcher launcher, Camera camera, Feeder feeder, FlywheelTable flyWheel) {
    m_launcher = launcher;
    m_camera = camera;
    m_feeder = feeder;
    m_flyWheelTable = flyWheel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_camera.setPipeline(0);
    var speed = m_flyWheelTable.getIdealTarget(-25).getSpeed();
    m_launcher.SetTargetRPM(speed);
    m_feeder.Stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_camera.hasTarget()) {
      var distance = m_camera.getDistance();
      var speed = m_flyWheelTable.getIdealTarget(distance).getSpeed();
      m_launcher.SetTargetRPM(speed);

      if (m_launcher.isAtTargetSpeed(speed)) {
        m_feeder.Both();
      } else {
        m_feeder.Stop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcher.coast();
    m_feeder.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
