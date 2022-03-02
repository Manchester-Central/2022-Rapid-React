// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public abstract class BaseRelativeDrive extends CommandBase {
  protected SwerveDrive m_drive;
  private Gamepad m_controller;

  /** Creates a new BaseRelativeDrive. */
  public BaseRelativeDrive(SwerveDrive drive, Gamepad controller) {
    m_controller = controller;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public final void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public final void execute() {
    // Absolute max meters per second = 3.8
    var maxMPS = 3.8;
    // Absolute max omegaradiant per second = 6.9 
    var maxOrPS = 8.75;
    var leftX = m_controller.getLeftX();
    var leftY = m_controller.getLeftY();
    var rightX = m_controller.getRightX();
    if(leftX == 0.0 && leftY == 0.0 && rightX == 0.0) {
      //m_drive.adjustToDefaultPosition();
      m_drive.stop();
    } else {
      moveRobot(leftX * maxMPS, leftY * maxMPS, rightX * maxOrPS);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public final void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public final boolean isFinished() {
    return false;
  }

  public abstract void moveRobot(double sidewaySpeed, double forwardSpeed, double thetaSpeed);
}
