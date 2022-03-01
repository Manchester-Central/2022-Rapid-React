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

public class Launcher extends SubsystemBase {
  private TalonFX m_ControllerA;
  private TalonFX m_ControllerB;

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
    m_ControllerA.config_kP(0, 0.05);
    m_ControllerB.config_kP(0, 0.05);
    m_ControllerA.config_kF(0, 0.05);
    m_ControllerB.config_kF(0, 0.05);

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
  }

  public void setTargetRpm(double rpm) {
    m_ControllerA.set(TalonFXControlMode.Velocity, rpm);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Launcher - Position", m_ControllerA.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Launcher - Speed", m_ControllerA.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Launcher - Error", m_ControllerA.getClosedLoopError(0));
    SmartDashboard.putNumber("Launcher - Target", m_ControllerA.getClosedLoopTarget(0));
    SmartDashboard.putString("Launcher - Control", m_ControllerA.getControlMode().toString());
    SmartDashboard.putNumber("Launcher - PowerOut", m_ControllerA.getMotorOutputPercent());
    // This method will be called once per scheduler run
  }

  public boolean isAtTargetSpeed(double targetRpm) {
    var currentRpm = m_ControllerA.getSelectedSensorVelocity();
    return currentRpm > targetRpm * 0.9 && currentRpm < targetRpm * 1.1;
  }
}
