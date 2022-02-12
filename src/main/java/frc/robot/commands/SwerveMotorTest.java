// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveModulePosition;

public class SwerveMotorTest extends CommandBase {
  private SwerveDrive m_drive;
  private Gamepad m_controller;
  private SwerveModulePosition m_moduleID;
  /** Creates a new SwerveMotorTest. */
  public SwerveMotorTest(SwerveDrive drive, Gamepad controller, SwerveModulePosition moduleID) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_controller = controller;
    m_moduleID = moduleID;
    addRequirements(drive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setSwerveModuleManual(m_moduleID, m_controller.getLeftY(), m_controller.getRightX() * 0.3); 
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
