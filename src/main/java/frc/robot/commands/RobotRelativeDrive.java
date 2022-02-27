// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;
import frc.robot.subsystems.SwerveDrive;

public class RobotRelativeDrive extends BaseRelativeDrive {

  public RobotRelativeDrive(SwerveDrive drive, Gamepad controller) {
    super(drive, controller);
  }

  @Override
  public void moveRobot(double leftX, double leftY, double rightX) {
    m_drive.moveRobotRelative(-leftX * 3, leftY * 3, -rightX * 3);
  }
}
