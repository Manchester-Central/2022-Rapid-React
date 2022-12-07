// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.chaos131.gamepads.Gamepad;

import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class DriverRelativeAngleDrive extends BaseRelativeDrive {

    public DriverRelativeAngleDrive(SwerveDrive drive, Gamepad controller) {
        super(drive, controller);
      }
    
      public void execute() {
        var leftX = m_controller.getLeftX();
        var leftY = m_controller.getLeftY();
        var rightAngle = m_controller.getRightAngle();
        var rightMagnitude = m_controller.getRightMagnitude();
        var multiplier = IsSlowMode ? 0.5 : 1.0;
        m_drive.moveDriverRelativeRotation(leftX * multiplier, leftY * multiplier, rightAngle, rightMagnitude);
      }

    @Override
    public void moveRobot(double sidewaySpeed, double forwardSpeed, double thetaSpeed) {
        // TODO Auto-generated method stub
        
    }

}
