// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonFX m_IntakeController;
  /** Creates a new Intake. */
  public Intake() {
    m_IntakeController = new TalonFX(Constants.Intake);
    m_IntakeController.setNeutralMode(NeutralMode.Coast);
  }
  public void ManualIntake(double power) {
    m_IntakeController.set(TalonFXControlMode.PercentOutput, power);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
