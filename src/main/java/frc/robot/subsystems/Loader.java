// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Loader extends SubsystemBase {
  private TalonFX m_ControllerTop;
  private TalonFX m_ControllerBottom;
  /** Creates a new Loader. */
  public Loader() {
    m_ControllerTop = new TalonFX(Constants.LoaderTop);
    m_ControllerBottom = new TalonFX(Constants.LoaderBottom);
    m_ControllerTop.setNeutralMode(NeutralMode.Coast);
    m_ControllerBottom.setNeutralMode(NeutralMode.Coast);
  }
  public void ManualLoad(double powerTop, double powerBottom) {
    m_ControllerTop.set(TalonFXControlMode.PercentOutput, powerTop);
    m_ControllerBottom.set(TalonFXControlMode.PercentOutput, powerBottom);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
