// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Launcher extends SubsystemBase {
  private TalonFX m_ControllerA;
  private TalonFX m_ControllerB;

  private double velocityP;
  private double velocityI;
  private double velocityD;
  private double velocityF;

  private boolean m_enableTuningPIDs = true;

  /** Creates a new Launcher. */
  public Launcher() {
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

    velocityP = 0.1;
    velocityI = 0.0001;
    velocityD = 0.3;
    velocityF = 0.059; // k_f = (PERCENT_POWER X 1023) / OBSERVED_VELOCITY

    m_ControllerA.config_kP(0, velocityP);
    m_ControllerB.config_kP(0, velocityP);
    m_ControllerA.config_kI(0, velocityI);
    m_ControllerB.config_kI(0, velocityI);
    m_ControllerA.config_kD(0, velocityD);
    m_ControllerB.config_kD(0, velocityD);
    m_ControllerA.config_kF(0, velocityF);
    m_ControllerB.config_kF(0, velocityF);
    m_ControllerA.enableVoltageCompensation(true);
    m_ControllerB.enableVoltageCompensation(true);

    Robot.LogManager.addNumber("Launcher/Speed2", () -> m_ControllerA.getSelectedSensorVelocity());

    if (m_enableTuningPIDs) {
      SmartDashboard.putNumber("Flywheel/P", velocityP);
      SmartDashboard.putNumber("Flywheel/I", velocityI);
      SmartDashboard.putNumber("Flywheel/D", velocityD);
    }

  }

  public void ManualLaunch(double power) {
    m_ControllerA.set(TalonFXControlMode.PercentOutput, power);
    m_ControllerB.set(TalonFXControlMode.PercentOutput, power);
  }

  public void SetTargetRPM(double rpm) {
    m_ControllerA.set(TalonFXControlMode.Velocity, rpm);
    m_ControllerB.set(TalonFXControlMode.Velocity, rpm);
  }

  public void coast() {
    m_ControllerA.set(TalonFXControlMode.PercentOutput, 0);
    m_ControllerB.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Launcher/Speed", m_ControllerA.getSelectedSensorVelocity(0));
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
      m_ControllerA.config_kP(0, velocityP);
      m_ControllerB.config_kP(0, velocityP);
      m_ControllerA.config_kI(0, velocityI);
      m_ControllerB.config_kI(0, velocityI);
      m_ControllerA.config_kD(0, velocityD);
      m_ControllerB.config_kD(0, velocityD);
      m_ControllerA.config_kF(0, velocityF);
      m_ControllerB.config_kF(0, velocityF);
    }
  }

  public boolean isAtTargetSpeed(double targetRpm) {
    var currentRpm = m_ControllerA.getSelectedSensorVelocity();
    return currentRpm > targetRpm * 0.90 && currentRpm < targetRpm * 1.10;
  }
}
