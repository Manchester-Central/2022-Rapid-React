// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
  private Field2d m_field = new Field2d();
  double m_x = 5;
  double m_y = 5;
  Rotation2d m_rotation = new Rotation2d();
  SwerveDriveModule m_module1;
  SwerveDriveModule m_module2;
  SwerveDriveModule m_module3;
  SwerveDriveModule m_module4;
  SwerveDriveKinematics m_kinematics;
  SwerveDriveOdometry m_odometry;
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    SmartDashboard.putData("Field", m_field);
    m_module1 = new SwerveDriveModule(-0.5, 0.5, "F_L");
    m_module2 = new SwerveDriveModule(0.5, 0.5, "F_R");
    m_module3 = new SwerveDriveModule(0.5, -0.5, "B_R");
    m_module4 = new SwerveDriveModule(-0.5, -0.5, "B_L");
    m_kinematics = new SwerveDriveKinematics(m_module1.getLocation(), m_module2.getLocation(), m_module3.getLocation(), m_module4.getLocation());
    m_odometry = new SwerveDriveOdometry(m_kinematics, m_rotation);
  }


  public void moveFieldRelative(double x, double y, double theta) {
   ChassisSpeeds speeds =  ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, m_rotation);
SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
m_module1.setTargetState(states[0]);
m_module2.setTargetState(states[1]);
m_module3.setTargetState(states[2]);
m_module4.setTargetState(states[3]);
  }
private Pose2d getPosition(){
return new Pose2d(m_x, m_y, m_rotation);
}

  @Override
  public void periodic() {
ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(m_module1.getState(), m_module2.getState(), m_module3.getState(), m_module4.getState());
m_rotation = m_rotation.plus(new Rotation2d(speeds.omegaRadiansPerSecond/Constants.RobotUpdate_hz));
    // This method will be called once per scheduler run
    m_odometry.update(m_rotation, m_module1.getState(), m_module2.getState(), m_module3.getState(), m_module4.getState());
    Pose2d Position = m_odometry.getPoseMeters();
    
    m_module1.updatePosition(Position);
    m_module2.updatePosition(Position);
    m_module3.updatePosition(Position);
    m_module4.updatePosition(Position);
    Position = Position.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    m_field.setRobotPose(Position);
    
  }
}

