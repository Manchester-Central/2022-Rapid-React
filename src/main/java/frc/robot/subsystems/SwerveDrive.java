// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  private Field2d m_field = new Field2d();
  double m_x = 5;
  double m_y = 5;
  Rotation2d m_rotation = new Rotation2d();
  SwerveDriveModule m_module1;
  SwerveDriveModule m_module2;
  SwerveDriveModule m_module3;
  SwerveDriveModule m_module4;
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    SmartDashboard.putData("Field", m_field);
    m_module1 = new SwerveDriveModule(-0.5, 0.5, "F_R");
    m_module2 = new SwerveDriveModule(0.5, 0.5, "F_L");
    m_module3 = new SwerveDriveModule(0.5, -0.5, "B_R");
    m_module4 = new SwerveDriveModule(-0.5, -0.5, "B_L");
  }

  public void moveFieldRelative(double x, double y, double theta) {
    m_rotation = new Rotation2d(m_rotation.getRadians() + theta);
    m_x = m_x + x;
    m_y = m_y + y;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_field.setRobotPose(m_x, m_y, m_rotation);
  }
}

