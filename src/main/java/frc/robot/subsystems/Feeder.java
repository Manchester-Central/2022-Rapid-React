// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private TalonFX m_upperFeeder;
  private TalonFX m_lowerFeeder;
  private DigitalInput m_beamSensor;
  private double k_intakeSpeed = 0.2;

  /** Creates a new Feeder. */
  public Feeder() {
    m_upperFeeder = new TalonFX(Constants.UpperFeeder);
    m_lowerFeeder = new TalonFX(Constants.LowerFeeder);
    m_upperFeeder.setNeutralMode(NeutralMode.Coast);
    m_lowerFeeder.setNeutralMode(NeutralMode.Coast);
    m_beamSensor = new DigitalInput(Constants.FeederBeamSensor);
  }

  public void ManualFeed(double powerTop, double powerBottom) {
    m_upperFeeder.set(TalonFXControlMode.PercentOutput, powerTop);
    m_lowerFeeder.set(TalonFXControlMode.PercentOutput, powerBottom);
  }

  public void Stop() {
    m_upperFeeder.set(TalonFXControlMode.PercentOutput, 0);
    m_lowerFeeder.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void Both() {
    m_upperFeeder.set(TalonFXControlMode.PercentOutput, k_intakeSpeed);
    m_lowerFeeder.set(TalonFXControlMode.PercentOutput, k_intakeSpeed);
  }
  
  public void Bottom() {
    m_upperFeeder.set(TalonFXControlMode.PercentOutput, 0);
    m_lowerFeeder.set(TalonFXControlMode.PercentOutput, k_intakeSpeed);
  }

  public boolean IsBallAtTopFeeder() {
    return m_beamSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
