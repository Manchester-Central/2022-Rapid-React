// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveModulePosition;

public class SwerveModuleTestDashboard extends CommandBase {
  private SwerveDrive m_drive;
  private SwerveModulePosition m_moduleID;

  public SwerveModuleTestDashboard(SwerveDrive drive, SwerveModulePosition moduleID) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_moduleID = moduleID;
    addRequirements(drive);
    initializeDashboardForModule();
  }

  @Override
  public void initialize() {
    initializeDashboardForModule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setSwerveModuleState(m_moduleID, getModuleState());

  }

  private void initializeDashboardForModule() {
    SmartDashboard.putNumber("swerve-test/" + m_moduleID.name() + "/angle", 0);
    SmartDashboard.putNumber("swerve-test/" + m_moduleID.name() + "/speed", 0);
  }

  private SwerveModuleState getModuleState() {
    var angle = SmartDashboard.getNumber("swerve-test/" + m_moduleID.name() + "/angle", 0);
    var speed = SmartDashboard.getNumber("swerve-test/" + m_moduleID.name() + "/speed", 0);
    return new SwerveModuleState(speed, Rotation2d.fromDegrees(angle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
