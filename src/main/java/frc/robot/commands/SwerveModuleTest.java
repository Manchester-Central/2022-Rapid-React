// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class SwerveModuleTest extends CommandBase {
  private SwerveDrive m_drive;
  private Gamepad m_controller;
  private int m_moduleID;

  public SwerveModuleTest(SwerveDrive drive, Gamepad controller, int moduleID) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_controller = controller;
    m_moduleID = moduleID;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = Math.toDegrees(Math.atan2(m_controller.getRightY(), m_controller.getRightX()));
    SwerveModuleState target = new SwerveModuleState(m_controller.getLeftY(), Rotation2d.fromDegrees(angle));
    m_drive.setSwerveModuleState(m_moduleID, target);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
