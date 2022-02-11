// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private TalonFX m_extensionController;
  private DoubleSolenoid m_solenoid;

  /** Creates a new Climber. */
  public Climber() {
    m_extensionController = new TalonFX(Constants.ClimberExtension);
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ClimberSolenoidForward, Constants.ClimberSolenoidReverse);
  }

  public void ManualExtend(double power) {
    m_extensionController.set(TalonFXControlMode.PercentOutput, power);
  }

  public void MoveArmUp() {
    m_solenoid.set(Value.kForward);
  }

  public void MoveArmDown() {
    m_solenoid.set(Value.kReverse);
  }

  public void ReleaseArm() {
    m_solenoid.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
