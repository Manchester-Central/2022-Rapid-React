// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public abstract class BaseRelativeDrive extends CommandBase {
  public static boolean IsSlowMode = false;

  protected SwerveDrive m_drive;
  protected Gamepad m_controller;
  private boolean m_alwaysMove;

  /** Creates a new BaseRelativeDrive. */
  public BaseRelativeDrive(SwerveDrive drive, Gamepad controller) {
    this(drive, controller, false);
  }

  public BaseRelativeDrive(SwerveDrive drive, Gamepad controller, boolean alwaysMove) {
    m_alwaysMove = alwaysMove;
    m_controller = controller;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var leftX = m_controller.getLeftX();
    var leftY = m_controller.getLeftY();
    var rightX = m_controller.getRightX();
    var multiplier = IsSlowMode ? 0.5 : 1.0;
    if(leftX == 0.0 && leftY == 0.0 && rightX == 0.0 && !m_alwaysMove) {
      //m_drive.adjustToDefaultPosition();
      m_drive.stop();
    } else {
      moveRobot(
        leftX * Constants.MaxMPS * multiplier,
        leftY * Constants.MaxMPS * multiplier,
        rightX * Constants.MaxORPS * multiplier
      );
    }
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

  public abstract void moveRobot(double sidewaySpeed, double forwardSpeed, double thetaSpeed);
}
