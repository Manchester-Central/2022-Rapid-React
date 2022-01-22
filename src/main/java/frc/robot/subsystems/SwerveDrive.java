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
  private Rotation2d m_rotation = new Rotation2d();
  private SwerveDriveModule m_module1;
  private SwerveDriveModule m_module2;
  private SwerveDriveModule m_module3;
  private SwerveDriveModule m_module4;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    SmartDashboard.putData("Field", m_field);
    m_module1 = new SwerveDriveModule(-0.5, 0.5, "F_L", Constants.SwerveFrontLeftVelocity, Constants.SwerveFrontLeftAngle);
    m_module2 = new SwerveDriveModule(0.5, 0.5, "F_R", Constants.SwerveFrontRightVelocity, Constants.SwerveFrontRightAngle);
    m_module3 = new SwerveDriveModule(0.5, -0.5, "B_R", Constants.SwerveBackRightVelocity, Constants.SwerveBackRightAngle);
    m_module4 = new SwerveDriveModule(-0.5, -0.5, "B_L", Constants.SwerveBackLeftVelocity, Constants.SwerveBackLeftAngle);
    m_kinematics = new SwerveDriveKinematics(m_module1.getLocation(), m_module2.getLocation(), m_module3.getLocation(),
        m_module4.getLocation());
    m_odometry = new SwerveDriveOdometry(m_kinematics, m_rotation);
  }

  private void move(ChassisSpeeds speeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    m_module1.setTargetState(states[0]);
    m_module2.setTargetState(states[1]);
    m_module3.setTargetState(states[2]);
    m_module4.setTargetState(states[3]);
  }

  public void moveFieldRelative(double x, double y, double theta) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, m_rotation);
    move(speeds);
  }

  public void moveDriverRelative(double x, double y, double theta) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(y, x * -1, theta, m_rotation);
    move(speeds);
  }

  public void moveRobotRelative(double x, double y, double theta) {
    ChassisSpeeds speeds = new ChassisSpeeds (x, y, theta);
    move(speeds);
  }
 
  public void setSwerveModuleState(SwerveModuleState module1State) {
    m_module1.setTargetState(module1State);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {
        m_module1.getState(), m_module2.getState(), m_module3.getState(), m_module4.getState()
    };
    return states;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(getModuleStates());
    m_rotation = m_rotation.plus(new Rotation2d(speeds.omegaRadiansPerSecond / Constants.RobotUpdate_hz));
    m_odometry.update(m_rotation, getModuleStates());
    Pose2d pose = getPose();
    m_module1.updatePosition(pose);
    m_module2.updatePosition(pose);
    m_module3.updatePosition(pose);
    m_module4.updatePosition(pose);
    pose = pose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    m_field.setRobotPose(pose);
    m_module1.periodic();
    m_module2.periodic();
    m_module3.periodic();
    m_module4.periodic();
  }
}
