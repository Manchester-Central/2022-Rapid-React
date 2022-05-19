// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TalonFxCHAOS;

public class Intake extends SubsystemBase {
  private TalonFxCHAOS m_IntakeController;
  private DoubleSolenoid m_solenoid;
  /** Creates a new Intake. */
  public Intake() {
    m_IntakeController = new TalonFxCHAOS(Constants.Intake, "Intake", "Main");
    m_IntakeController.setNeutralMode(NeutralMode.Coast);
    m_IntakeController.configOpenloopRamp(0.4);
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.IntakeSolenoidForward, Constants.IntakeSolenoidReverse);

    // Lower CAN Utilization. We are only setting percent power to the motor, so we don't need the data
    m_IntakeController.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Constants.MaxCANStatusFramePeriod);
    m_IntakeController.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Constants.MaxCANStatusFramePeriod);
    
  }
  public void ManualIntake(double power) {
    if (m_solenoid.get() == Value.kForward) {
      power = 0;
    }
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

}
