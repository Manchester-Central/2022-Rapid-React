// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
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
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrive extends SubsystemBase {
  private Field2d m_field = new Field2d();
  private SwerveDriveModule m_module1;
  private SwerveDriveModule m_module2;
  private SwerveDriveModule m_module3;
  private SwerveDriveModule m_module4;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;
  private AHRS m_gyro;

  private double velocityP;
  private double velocityI;
  private double velocityD;
  private double angleP;
  private double angleI;
  private double angleD;

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(m_gyro.getYaw());
  }

  private void setSimulationAngle(double angle) {
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble SimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    SimAngle.set(angle);
  }

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    SmartDashboard.putData("Field", m_field);
    m_module1 = new SwerveDriveModule(-0.5, 0.5, 0, "F_L", Constants.SwerveFrontLeftVelocity,
        Constants.SwerveFrontLeftAngle);
    m_module2 = new SwerveDriveModule(0.5, 0.5, -90, "F_R", Constants.SwerveFrontRightVelocity,
        Constants.SwerveFrontRightAngle);
    m_module3 = new SwerveDriveModule(0.5, -0.5, 180, "B_R", Constants.SwerveBackRightVelocity,
        Constants.SwerveBackRightAngle);
    m_module4 = new SwerveDriveModule(-0.5, -0.5, 90, "B_L", Constants.SwerveBackLeftVelocity,
        Constants.SwerveBackLeftAngle);
    m_kinematics = new SwerveDriveKinematics(m_module1.getLocation(), m_module2.getLocation(), m_module3.getLocation(),
        m_module4.getLocation());
    m_gyro = new AHRS(SPI.Port.kMXP);
    m_odometry = new SwerveDriveOdometry(m_kinematics, getRotation());

    velocityP = 1;
    velocityI = 0;
    velocityD = 0;
    angleP = 1;
    angleI = 0;
    angleD = 0;
    updateVelocityPIDConstants(velocityP, velocityI, velocityD);
    updateAnglePIDConstants(angleP, angleI, angleD);
    
    SmartDashboard.putNumber("Velocity/P", velocityP);
    SmartDashboard.putNumber("Velocity/I", velocityI);
    SmartDashboard.putNumber("Velocity/D", velocityD);
    SmartDashboard.putNumber("Angle/P", angleP);
    SmartDashboard.putNumber("Angle/I", angleI);
    SmartDashboard.putNumber("Angle/D", angleD);
  }

  private void move(ChassisSpeeds speeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    m_module1.setTargetState(states[0]);
    m_module2.setTargetState(states[1]);
    m_module3.setTargetState(states[2]);
    m_module4.setTargetState(states[3]);
  }

  public void moveFieldRelative(double x, double y, double theta) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, getRotation());
    move(speeds);
  }

  public void moveDriverRelative(double x, double y, double theta) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(y, x * -1, theta, getRotation());
    move(speeds);
  }

  public void moveRobotRelative(double x, double y, double theta) {
    ChassisSpeeds speeds = new ChassisSpeeds(x, y, theta);
    move(speeds);
  }

  public void setSwerveModuleState(int moduleID, SwerveModuleState State) {
    m_module1.Stop();
    m_module2.Stop();
    m_module3.Stop();
    m_module4.Stop();
    switch (moduleID) {
      case 1:
        m_module1.setTargetState(State);
        break;
      case 2:
        m_module2.setTargetState(State);
        break;
      case 3:
        m_module3.setTargetState(State);
        break;
      case 4:
        m_module4.setTargetState(State);
        break;
      default:
        break;
    }
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
    m_odometry.update(getRotation(), getModuleStates());
    Pose2d pose = getPose();
    m_module1.updatePosition(pose);
    m_module2.updatePosition(pose);
    m_module3.updatePosition(pose);
    m_module4.updatePosition(pose);
    pose = pose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    m_field.setRobotPose(pose);
    SmartDashboard.putBoolean("calibrating", m_gyro.isCalibrating());

    double newVelocityP = SmartDashboard.getNumber("Velocity/P", velocityP);
    double newVelocityI = SmartDashboard.getNumber("Velocity/I", velocityI);
    double newVelocityD = SmartDashboard.getNumber("Velocity/D", velocityD);

    if (newVelocityP != velocityP || newVelocityI != velocityI || newVelocityD != velocityD) {
      velocityP = newVelocityP;
      velocityI = newVelocityI;
      velocityD = newVelocityD;
      updateVelocityPIDConstants(velocityP, velocityI, velocityD);
    }

    double newAngleP = SmartDashboard.getNumber("Angle/P", angleP);
    double newAngleI = SmartDashboard.getNumber("Angle/I", angleI);
    double newAngleD = SmartDashboard.getNumber("Angle/D", angleD);

    if (newAngleP != angleP || newAngleI != angleI || newAngleD != angleD) {
      angleP = newAngleP;
      angleI = newAngleI;
      angleD = newAngleD;
      updateAnglePIDConstants(angleP, angleI, angleD);
    }
  }

  private void updateVelocityPIDConstants(double P, double I, double D) {
    m_module1.UpdateVelocityPIDConstants(P, I, D);
    m_module2.UpdateVelocityPIDConstants(P, I, D);
    m_module3.UpdateVelocityPIDConstants(P, I, D);
    m_module4.UpdateVelocityPIDConstants(P, I, D);
  }

  private void updateAnglePIDConstants(double P, double I, double D) {
    m_module1.UpdateAnglePIDConstants(P, I, D);
    m_module2.UpdateAnglePIDConstants(P, I, D);
    m_module3.UpdateAnglePIDConstants(P, I, D);
    m_module4.UpdateAnglePIDConstants(P, I, D);
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(getModuleStates());
    setSimulationAngle(
        getRotation().plus(new Rotation2d(speeds.omegaRadiansPerSecond / Constants.RobotUpdate_hz)).getDegrees());
  }

}
