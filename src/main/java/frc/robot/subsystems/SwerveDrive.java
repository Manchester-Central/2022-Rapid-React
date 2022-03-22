// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Robot;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrive extends SubsystemBase {
  private final boolean m_enableTuningPIDs = false;
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

  private double translationP;
  private double translationI;
  private double translationD;

  private double rotationP;
  private double rotationI;
  private double rotationD;

  private PIDController m_xTranslationPID;
  private PIDController m_yTranslationPID;
  private PIDController m_rotationPID;


  public enum SwerveModulePosition {
    FrontLeft, FrontRight, BackLeft, BackRight
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());

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

    velocityP = 0.1;
    velocityI = 0;
    velocityD = 0;
    angleP = 0.2;
    angleI = 0;
    angleD = 0;
    updateVelocityPIDConstants(velocityP, velocityI, velocityD);
    updateAnglePIDConstants(angleP, angleI, angleD);

    translationP = 0.1;
    translationI = 0.0;
    translationD = 0.0;
    m_xTranslationPID = new PIDController(translationP, translationI, translationD);
    m_yTranslationPID = new PIDController(translationP, translationI, translationD);
    m_xTranslationPID.setTolerance(0.01);
    m_yTranslationPID.setTolerance(0.01);

    rotationP = 0.1;
    rotationI = 0.0;
    rotationD = 0.0;
    m_rotationPID = new PIDController(rotationP, rotationI, rotationD);
    m_rotationPID.setTolerance(0.5); 

    if (m_enableTuningPIDs) {
      SmartDashboard.putNumber("Velocity/P", velocityP);
      SmartDashboard.putNumber("Velocity/I", velocityI);
      SmartDashboard.putNumber("Velocity/D", velocityD);
      SmartDashboard.putNumber("Angle/P", angleP);
      SmartDashboard.putNumber("Angle/I", angleI);
      SmartDashboard.putNumber("Angle/D", angleD);
      SmartDashboard.putNumber("Translation/P", translationP);
      SmartDashboard.putNumber("Translation/I", translationI);
      SmartDashboard.putNumber("Translation/D", translationD);
      SmartDashboard.putNumber("Rotation/P", rotationP);
      SmartDashboard.putNumber("Rotation/I", rotationI);
      SmartDashboard.putNumber("Rotation/D", rotationD);
    }

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
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(sidewaysSpeed, forwardSpeed, theta, getRotation());
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
    Pose2d correctedPose = new Pose2d(pose.getX(), -pose.getY(), pose.getRotation());
    m_moduleFL.updatePosition(correctedPose);
    m_moduleFR.updatePosition(correctedPose);
    m_moduleBR.updatePosition(correctedPose);
    m_moduleBL.updatePosition(correctedPose);
    //pose = pose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    m_field.setRobotPose(correctedPose);
    SmartDashboard.putBoolean("Gyro/Calibrating", m_gyro.isCalibrating());
    SmartDashboard.putNumber("Gyro/Angle", getRotation().getDegrees());

    if (m_enableTuningPIDs) {
      tunePIDs();
    }
  }

  private void tunePIDs() {
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

    double newTranslationP = SmartDashboard.getNumber("Translation/P", translationP);
    double newTranslationI = SmartDashboard.getNumber("Translation/I", translationI);
    double newTranslationD = SmartDashboard.getNumber("Translation/D", translationD);

    if (newTranslationP != translationP || newTranslationI != translationI || newTranslationD != translationD) {
      translationP = newTranslationP;
      translationI = newTranslationI;
      translationD = newTranslationD;
      m_xTranslationPID.setPID(translationP, translationI, translationD);
      m_yTranslationPID.setPID(translationP, translationI, translationD);
    }

    double newRotationP = SmartDashboard.getNumber("Rotation/P", rotationP);
    double newRotationI = SmartDashboard.getNumber("Rotation/I", rotationI);
    double newRotationD = SmartDashboard.getNumber("Rotation/D", rotationD);

    if (newRotationP != rotationP || newRotationI != rotationI || newRotationD != rotationD) {
      rotationP = newRotationP;
      rotationI = newRotationI;
      rotationD = newRotationD;
      m_rotationPID.setPID(rotationP, rotationI, rotationD); 
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

  public void setTargetPose(Pose2d targetPose) {
    m_xTranslationPID.setSetpoint(targetPose.getX());
    m_yTranslationPID.setSetpoint(targetPose.getY());
    m_rotationPID.setSetpoint(AngleUtil.closestTarget(getRotation().getDegrees(), targetPose.getRotation().getDegrees()));
  }

  public void driveToPosition() {
    var currentPose = getPose();
    var vx = m_xTranslationPID.calculate(currentPose.getX());
    var vy = m_yTranslationPID.calculate(currentPose.getY());
    var theta = m_rotationPID.calculate(currentPose.getRotation().getDegrees());
    var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx * Constants.MaxMPS, vy * Constants.MaxMPS, theta * Constants.MaxORPS, getRotation());
    move(speeds);
  }

  public boolean isAtTargetPose() {
    return m_xTranslationPID.atSetpoint() && m_yTranslationPID.atSetpoint() && m_rotationPID.atSetpoint();
  }

  public void deleteDriveToPositionError() {
    m_xTranslationPID.reset();
    m_yTranslationPID.reset();
    m_rotationPID.reset();
  }

}