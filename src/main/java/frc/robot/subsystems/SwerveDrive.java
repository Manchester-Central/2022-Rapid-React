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
  private SwerveDriveModule m_moduleFL;
  private SwerveDriveModule m_moduleFR;
  private SwerveDriveModule m_moduleBR;
  private SwerveDriveModule m_moduleBL;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;
  private AHRS m_gyro;

  private double velocityP;
  private double velocityI;
  private double velocityD;
  private double angleP;
  private double angleI;
  private double angleD;

  public enum SwerveModulePosition {
    FrontLeft, FrontRight, BackLeft, BackRight
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(m_gyro.getAngle());

  }

  public void updateGyroAdjustmentAngle(double physicalAngle) {
    double sensorAngle = m_gyro.getYaw();
    double angleOffset = physicalAngle - sensorAngle;
    m_gyro.setAngleAdjustment(angleOffset);

  }

  private void setSimulationAngle(double angle) {
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble SimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    SimAngle.set(angle);
  }

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    SmartDashboard.putData("Field", m_field);
    double width = 0.2957;
    double length = 0.32067;
    m_moduleFL = new SwerveDriveModule(length, -width, SwerveModulePosition.FrontLeft.name(),
        Constants.SwerveFrontLeftVelocity,
        Constants.SwerveFrontLeftAngle, Constants.SwerveFrontLeftAbsolute, -110.5);
    m_moduleFR = new SwerveDriveModule(length, width, SwerveModulePosition.FrontRight.name(),
        Constants.SwerveFrontRightVelocity,
        Constants.SwerveFrontRightAngle, Constants.SwerveFrontRightAbsolute, 173.0);
    m_moduleBL = new SwerveDriveModule(-length, -width, SwerveModulePosition.BackLeft.name(),
        Constants.SwerveBackLeftVelocity,
        Constants.SwerveBackLeftAngle, Constants.SwerveBackLeftAbsolute, 236.6);
    m_moduleBR = new SwerveDriveModule(-length, width, SwerveModulePosition.BackRight.name(),
        Constants.SwerveBackRightVelocity,
        Constants.SwerveBackRightAngle, Constants.SwerveBackRightAbsolute, -15.7);
    m_kinematics = new SwerveDriveKinematics(m_moduleFL.getLocation(), m_moduleFR.getLocation(),
        m_moduleBR.getLocation(),
        m_moduleBL.getLocation());
    m_gyro = new AHRS(SPI.Port.kMXP);
    m_odometry = new SwerveDriveOdometry(m_kinematics, getRotation());

    velocityP = 0.1;
    velocityI = 0;
    velocityD = 0;
    angleP = 0.2;
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

  public void stop() {
    m_moduleFR.Stop();
    m_moduleFL.Stop();
    m_moduleBR.Stop();
    m_moduleBL.Stop();
  }

  public void ResetEncoders() {
    m_moduleFR.ResetEncoders();
    m_moduleFL.ResetEncoders();
    m_moduleBR.ResetEncoders();
    m_moduleBL.ResetEncoders();
  }

  public void adjustToDefaultPosition() {
    m_moduleFL.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_moduleFR.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_moduleBR.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_moduleBL.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  private void move(ChassisSpeeds speeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    m_moduleFL.setTargetState(states[0]);
    m_moduleFR.setTargetState(states[1]);
    m_moduleBR.setTargetState(states[2]);
    m_moduleBL.setTargetState(states[3]);
  }

  public void moveFieldRelative(double sidewaysSpeed, double forwardSpeed, double theta) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, sidewaysSpeed, theta, getRotation());
    move(speeds);
  }

  public void moveDriverRelative(double sidewaysSpeed, double forwardSpeed, double theta) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, sidewaysSpeed * -1, theta,
        getRotation());
    move(speeds);
  }

  public void moveRobotRelative(double sidewaysSpeed, double forwardSpeed, double theta) {
    ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, sidewaysSpeed, theta);
    move(speeds);
  }

  public void setSwerveModuleState(SwerveModulePosition moduleID, SwerveModuleState State) {
    m_moduleFL.Stop();
    m_moduleFR.Stop();
    m_moduleBR.Stop();
    m_moduleBL.Stop();
    switch (moduleID) {
      case FrontLeft:
        m_moduleFL.setTargetState(State);
        break;
      case FrontRight:
        m_moduleFR.setTargetState(State);
        break;
      case BackRight:
        m_moduleBR.setTargetState(State);
        break;
      case BackLeft:
        m_moduleBL.setTargetState(State);
        break;
      default:
        break;
    }
  }

  public void setSwerveModuleManual(SwerveModulePosition moduleID, double velocityControllerPower,
      double angleControllerPower) {
    m_moduleFL.Stop();
    m_moduleFR.Stop();
    m_moduleBR.Stop();
    m_moduleBL.Stop();
    switch (moduleID) {
      case FrontLeft:
        m_moduleFL.setManual(velocityControllerPower, angleControllerPower);
        break;
      case FrontRight:
        m_moduleFR.setManual(velocityControllerPower, angleControllerPower);
        break;
      case BackRight:
        m_moduleBR.setManual(velocityControllerPower, angleControllerPower);
        break;
      case BackLeft:
        m_moduleBL.setManual(velocityControllerPower, angleControllerPower);
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
        m_moduleFL.getState(), m_moduleFR.getState(), m_moduleBR.getState(), m_moduleBL.getState()
    };
    return states;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(getRotation(), getModuleStates());
    Pose2d pose = getPose();
    m_moduleFL.updatePosition(pose);
    m_moduleFR.updatePosition(pose);
    m_moduleBR.updatePosition(pose);
    m_moduleBL.updatePosition(pose);
    //pose = pose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    m_field.setRobotPose(pose);
    SmartDashboard.putBoolean("Gyro - calibrating", m_gyro.isCalibrating());
    SmartDashboard.putNumber("Gyro - Yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("Gyro - Angle (adjusted)", m_gyro.getAngle());

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
    m_moduleFL.UpdateVelocityPIDConstants(P, I, D);
    m_moduleFR.UpdateVelocityPIDConstants(P, I, D);
    m_moduleBR.UpdateVelocityPIDConstants(P, I, D);
    m_moduleBL.UpdateVelocityPIDConstants(P, I, D);
  }

  private void updateAnglePIDConstants(double P, double I, double D) {
    m_moduleFL.UpdateAnglePIDConstants(P, I, D);
    m_moduleFR.UpdateAnglePIDConstants(P, I, D);
    m_moduleBR.UpdateAnglePIDConstants(P, I, D);
    m_moduleBL.UpdateAnglePIDConstants(P, I, D);
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(getModuleStates());
    var currentYaw = m_gyro.getYaw();
    setSimulationAngle( 
        currentYaw + new Rotation2d(speeds.omegaRadiansPerSecond / Constants.RobotUpdate_hz).getDegrees());
  }

}
