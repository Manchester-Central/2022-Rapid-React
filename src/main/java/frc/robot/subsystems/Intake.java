// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonFX m_IntakeController;
  private DoubleSolenoid m_solenoid;
  /** Creates a new Intake. */
  public Intake() {
    m_IntakeController = new TalonFX(Constants.Intake);
    m_IntakeController.setNeutralMode(NeutralMode.Coast);
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.IntakeSolenoidForward, Constants.IntakeSolenoidReverse);
    
  }
  public void ManualIntake(double power) {
    m_IntakeController.set(TalonFXControlMode.PercentOutput, power);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void MoveIntakeUp() {
    m_solenoid.set(Value.kForward);
  }

  public void MoveIntakeDown() {
    m_solenoid.set(Value.kReverse);
  }

  public void ReleaseIntake() {
    m_solenoid.set(Value.kOff);
  }
}
