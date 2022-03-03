// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.KeyStore.SecretKeyEntry;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private DigitalInput m_limitSwitch;
  private TalonFX m_extensionController;
  private DoubleSolenoid m_solenoid;
  private double m_downPositionCounts;
  private boolean m_seenBottom;

  /** Creates a new Climber. */
  public Climber() {
    m_extensionController = new TalonFX(Constants.ClimberExtension);
    m_extensionController.setNeutralMode(NeutralMode.Brake);
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ClimberSolenoidForward, Constants.ClimberSolenoidReverse);
    m_limitSwitch = new DigitalInput(Constants.ExtenderLimitSwitch);
    m_extensionController.config_kP(0, 0.1);
  }

  public boolean hasSeenBottom() {
    return m_seenBottom;
  }

  public boolean isCLimberAtBottom() {
    return !m_limitSwitch.get();
  }

  public void ManualExtend(double power) {
    if (isCLimberAtBottom()) {
      if (power < 0) {
        power = 0;
      }
    }
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

  public void ExtendToTop() {
    if(m_seenBottom) {
      m_extensionController.set(TalonFXControlMode.Position, m_downPositionCounts + 400000);
    }
  }

  public void ExtendToBottom() {
    if(m_seenBottom && !isCLimberAtBottom()) {
      m_extensionController.set(TalonFXControlMode.Position, m_downPositionCounts);
    }
  }

  @Override
  public void periodic() {
    if (!m_seenBottom) {
      if (isCLimberAtBottom()) {
        m_downPositionCounts = m_extensionController.getSelectedSensorPosition();
        m_seenBottom = true;
      }
    }
    SmartDashboard.putBoolean("Climber/At Bottom", isCLimberAtBottom());
  }
}
