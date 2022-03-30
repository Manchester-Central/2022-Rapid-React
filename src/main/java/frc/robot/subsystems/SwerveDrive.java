// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.RobotBase;
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

  private PIDController m_xTranslationPID;
  private PIDController m_yTranslationPID;
  private PIDController m_rotationPID;
  private PIDController m_rotationAutoPID;

  private PIDTuner m_xTranslationPIDTuner;
  private PIDTuner m_yTranslationPIDTuner;
  private PIDTuner m_rotationPIDTuner;
  private PIDTuner m_rotationAutoPIDTuner;
  private PIDTuner m_moduleVelocityPIDTuner;
  private PIDTuner m_moduleAnglePIDTuner;

  public enum SwerveModulePosition {
    FrontLeft, FrontRight, BackLeft, BackRight
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(AngleUtil.clampAngle(-m_gyro.getAngle()));

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
    Robot.LogManager.addNumber("Swerve/X", () -> m_odometry.getPoseMeters().getX());
    Robot.LogManager.addNumber("Swerve/Y", () -> m_odometry.getPoseMeters().getY());

    double velocityP = 0.1;
    double velocityI = 0;
    double velocityD = 0;
    // `this::updateVelocityPIDConstants` is basically shorthand for `(PIDUpdate update) -> updateVelocityPIDConstants(update)`
    m_moduleVelocityPIDTuner = new PIDTuner("Swerve/ModuleVelocity", Robot.EnablePIDTuning, velocityP, velocityI, velocityD, this::updateVelocityPIDConstants);
    double angleP = 0.2;
    double angleI = 0;
    double angleD = 0;
    m_moduleAnglePIDTuner = new PIDTuner("Swerve/ModuleAngle", Robot.EnablePIDTuning, angleP, angleI, angleD, this::updateAnglePIDConstants);

    double translationP = 1.0;
    double translationI = 0.0;
    double translationD = 0.0;
    m_xTranslationPID = new PIDController(translationP, translationI, translationD);
    m_yTranslationPID = new PIDController(translationP, translationI, translationD);
    m_xTranslationPID.setTolerance(0.01);
    m_yTranslationPID.setTolerance(0.01);
    m_xTranslationPIDTuner = new PIDTuner("Swerve/xTranslation", Robot.EnablePIDTuning, m_xTranslationPID);
    m_yTranslationPIDTuner = new PIDTuner("Swerve/yTranslation", Robot.EnablePIDTuning, m_yTranslationPID);

    double rotationP = 0.017;
    double rotationI = 0.0001;
    double rotationD = 0.0;
    m_rotationPID = new PIDController(rotationP, rotationI, rotationD);
    m_rotationPIDTuner = new PIDTuner("Swerve/Rotation", Robot.EnablePIDTuning, m_rotationPID);
    m_rotationPID.setTolerance(3.0);
    m_rotationPID.enableContinuousInput(-180, 180);

    double rotationAutoP = 0.01;
    double rotationAutoI = 0.0001;
    double rotationAutoD = 0.0;
    m_rotationAutoPID = new PIDController(rotationAutoP, rotationAutoI, rotationAutoD);
    m_rotationAutoPIDTuner = new PIDTuner("Swerve/RotationAuto", Robot.EnablePIDTuning, m_rotationAutoPID);
    m_rotationAutoPID.setTolerance(3.0);
    m_rotationAutoPID.enableContinuousInput(-180, 180);

    Robot.LogManager.addNumber("Gyro/AccelX", m_gyro::getRawAccelX);
    Robot.LogManager.addNumber("Gyro/AccelY", m_gyro::getRawAccelY);
    Robot.LogManager.addNumber("Gyro/AccelZ", m_gyro::getRawAccelZ);
  }

  public void updateOdometry(double x, double y, double angle) {
    updateGyroAdjustmentAngle(angle);
    m_odometry.resetPosition(new Pose2d(x, y, getRotation()), getRotation());
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
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(sidewaysSpeed, forwardSpeed, theta, getRotation().times(-1));
    move(speeds);
  }

  public void moveDriverRelative(double sidewaysSpeed, double forwardSpeed, double theta) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, sidewaysSpeed * -1, theta,
        getRotation().times(-1));
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
    Pose2d correctedPose = new Pose2d(pose.getX(), -pose.getY(), pose.getRotation());
    m_moduleFL.updatePosition(correctedPose);
    m_moduleFR.updatePosition(correctedPose);
    m_moduleBR.updatePosition(correctedPose);
    m_moduleBL.updatePosition(correctedPose);
    // pose = pose.transformBy(new Transform2d(new Translation2d(),
    // Rotation2d.fromDegrees(90)));
    m_field.setRobotPose(correctedPose);
    SmartDashboard.putBoolean("Gyro/Calibrating", m_gyro.isCalibrating());
    SmartDashboard.putNumber("Gyro/Angle", getRotation().getDegrees());

    m_xTranslationPIDTuner.tune();
    m_yTranslationPIDTuner.tune();
    m_rotationPIDTuner.tune();
    m_rotationAutoPIDTuner.tune();
    m_moduleVelocityPIDTuner.tune();
    m_moduleAnglePIDTuner.tune();
  }

  private void updateVelocityPIDConstants(PIDUpdate update) {
    m_moduleFL.UpdateVelocityPIDConstants(update);
    m_moduleFR.UpdateVelocityPIDConstants(update);
    m_moduleBR.UpdateVelocityPIDConstants(update);
    m_moduleBL.UpdateVelocityPIDConstants(update);
  }

  private void updateAnglePIDConstants(PIDUpdate update) {
    m_moduleFL.UpdateAnglePIDConstants(update);
    m_moduleFR.UpdateAnglePIDConstants(update);
    m_moduleBR.UpdateAnglePIDConstants(update);
    m_moduleBL.UpdateAnglePIDConstants(update);
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(getModuleStates());
    var currentYaw = m_gyro.getYaw();
    setSimulationAngle(
        currentYaw - new Rotation2d(speeds.omegaRadiansPerSecond / Constants.RobotUpdate_hz).getDegrees());
  }

  public void setTargetPosition(double x, double y) {
    m_xTranslationPID.setSetpoint(x);
    m_yTranslationPID.setSetpoint(y);
  }

  public void setTargetAngle(Rotation2d targetAngle) {
    m_rotationPID.setSetpoint(targetAngle.getDegrees());
  }

  public boolean isAtTargetAngle() {
    return m_rotationPID.atSetpoint();
  }

  public void setTargetPose(Pose2d targetPose) {
    setTargetPosition(targetPose.getX(), targetPose.getY());
    m_rotationAutoPID.setSetpoint(targetPose.getRotation().getDegrees());
  }

  public double getTargetVx() {
    return MathUtil.clamp(m_xTranslationPID.calculate(getPose().getX()), -1, 1) * Constants.MaxMPS;
  }

  public double getTargetVy() {
    return MathUtil.clamp(m_yTranslationPID.calculate(getPose().getY()), -1, 1) * Constants.MaxMPS;
  }

  public double getTargetOmega() {
    return -MathUtil.clamp(m_rotationPID.calculate(getRotation().getDegrees()), -1, 1) * Constants.MaxORPS;
  }

  public double getTargetOmegaAuto() {
    return -MathUtil.clamp(m_rotationAutoPID.calculate(getRotation().getDegrees()), -1, 1) * Constants.MaxORPS;
  }

  public void driveToPosition() {
    var vx = getTargetVx();
    var vy = getTargetVy();
    var Omega = getTargetOmegaAuto();
    var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, Omega, getRotation());
    move(speeds);
  }

  public boolean isAtTargetPose() {
    return m_xTranslationPID.atSetpoint() && m_yTranslationPID.atSetpoint() && m_rotationAutoPID.atSetpoint();
  }

  public void deleteDriveToPositionError() {
    m_xTranslationPID.reset();
    m_yTranslationPID.reset();
    m_rotationPID.reset();
    m_rotationAutoPID.reset();
  }

}