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
  private TalonFX m_ControllerA;
  private TalonFX m_ControllerB;
  /** Creates a new Loader. */
  public Loader() {
    m_ControllerA = new TalonFX(Constants.LoaderA);
    m_ControllerB = new TalonFX(Constants.LoaderB);
    m_ControllerA.setNeutralMode(NeutralMode.Coast);
    m_ControllerB.setNeutralMode(NeutralMode.Coast);
    m_ControllerB.follow(m_ControllerA);
    m_ControllerB.setInverted(InvertType.FollowMaster);
  }
  public void ManualLoad(double power) {
    m_ControllerA.set(TalonFXControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
