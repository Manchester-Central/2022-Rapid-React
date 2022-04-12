// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.pid.PIDTuner;
import com.chaos131.pid.PIDUpdate;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Launcher extends SubsystemBase {
  private DoubleSolenoid m_solenoid;
  private TalonFX m_ControllerA;
  private TalonFX m_ControllerB;

  private PIDTuner m_pidTuner;

  private double m_speedTolerance = Constants.DefaultLauncherTolerance;

  /** Creates a new Launcher. */
  public Launcher() {
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.LauncherSolenoidForward,
        Constants.LauncherSolenoidReverse);
    m_ControllerA = new TalonFX(Constants.LauncherA);
    m_ControllerB = new TalonFX(Constants.LauncherB);
    m_ControllerA.configOpenloopRamp(0.2);
    m_ControllerB.configOpenloopRamp(0.2);
    m_ControllerA.setNeutralMode(NeutralMode.Coast);
    m_ControllerB.setNeutralMode(NeutralMode.Coast);
    m_ControllerA.setInverted(InvertType.InvertMotorOutput);
    m_ControllerB.setInverted(InvertType.None);
    m_ControllerA.configPeakOutputReverse(0);
    m_ControllerB.configPeakOutputReverse(0);
    m_ControllerA.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
    m_ControllerB.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
    m_ControllerA.configVelocityMeasurementWindow(1);
    m_ControllerB.configVelocityMeasurementWindow(1);
    m_ControllerA.configVoltageCompSaturation(12.4);
    m_ControllerB.configVoltageCompSaturation(12.4);

    m_ControllerA.enableVoltageCompensation(true);
    m_ControllerB.enableVoltageCompensation(true);

    double velocityP = 0.075;
    double velocityI = 0.0003;
    double velocityD = 0.0;
    double velocityF = 0.023; // k_f = (PERCENT_POWER X 1023) / OBSERVED_VELOCITY
    m_pidTuner = new PIDTuner("Launcher", Robot.EnablePIDTuning, velocityP, velocityI, velocityD, velocityF, this::updatePIDF);

    Robot.LogManager.addNumber("Launcher/Speed2", () -> m_ControllerA.getSelectedSensorVelocity());
  }

  public void ManualLaunch(double power) {
    m_ControllerA.set(TalonFXControlMode.PercentOutput, power);
    m_ControllerB.set(TalonFXControlMode.PercentOutput, power);
  }

  public void SetTargetRPM(double rpm) {
    m_ControllerA.set(TalonFXControlMode.Velocity, rpm);
    m_ControllerB.set(TalonFXControlMode.Velocity, rpm);
  }

  public void spinUpSpeed() {
    if (DriverStation.isAutonomous()) {
      m_ControllerA.set(TalonFXControlMode.Velocity, Constants.DefaultLauncherSpinUpAuto);
      m_ControllerB.set(TalonFXControlMode.Velocity, Constants.DefaultLauncherSpinUpAuto);
    } else {
      m_ControllerA.set(TalonFXControlMode.Velocity, Constants.DefaultLauncherSpinUpTeleop);
      m_ControllerB.set(TalonFXControlMode.Velocity, Constants.DefaultLauncherSpinUpTeleop);
    }
  }

  public void coast() {
    m_ControllerA.set(TalonFXControlMode.PercentOutput, 0);
    m_ControllerB.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Launcher/Speed", m_ControllerA.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Launcher/TargetSpeed", m_ControllerA.getClosedLoopTarget(0));
    m_pidTuner.tune();
  }

  private void updatePIDF(PIDUpdate update) {
    m_ControllerA.config_kP(0, update.P);
    m_ControllerB.config_kP(0, update.P);
    m_ControllerA.config_kI(0, update.I);
    m_ControllerB.config_kI(0, update.I);
    m_ControllerA.config_kD(0, update.D);
    m_ControllerB.config_kD(0, update.D);
    m_ControllerA.config_kF(0, update.F);
    m_ControllerB.config_kF(0, update.F);
}

  public void MoveHoodUp() {
    m_solenoid.set(Value.kForward);
  }

  public void MoveHoodDown() {
    m_solenoid.set(Value.kReverse);
  }

  public void setLauncherTolerance(double tolerance) {
    m_speedTolerance = tolerance;
  }

  public boolean isAtTargetSpeed(double targetRpm) {
    var currentRpm = m_ControllerA.getSelectedSensorVelocity();
    return currentRpm > targetRpm - m_speedTolerance && currentRpm < targetRpm + m_speedTolerance;
  }
}
