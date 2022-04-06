// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.FlywheelTable;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SwerveDrive;

public class AutoAimLaunch extends DriverRelativeDriveAimAndLaunch {
  /** Creates a new AutoAimLaunch. */
  public AutoAimLaunch(SwerveDrive drive, Gamepad controller, Camera camera, Launcher launcher, 
    FlywheelTable flywheelTable, Feeder feeder) {

    super(drive, controller, camera, launcher, flywheelTable, feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_feeder.IsBallAtTopFeeder() && !m_feeder.IsBallAtMiddleFeeder();
  }
}
