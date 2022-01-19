// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class RobotRelativeDrive extends CommandBase {
  /** Creates a new RobotRelativeDrive. */
    private SwerveDrive m_drive;
    private Gamepad m_controller;

  public RobotRelativeDrive(SwerveDrive drive, Gamepad controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_controller = controller;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.moveRobotRelative(m_controller.getLeftX() * 3, m_controller.getLeftY() * 3, m_controller.getRightX() * -3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
