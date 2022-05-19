// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.KeyStore.SecretKeyEntry;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TalonFxCHAOS;

public class Climber extends SubsystemBase {
  private DigitalInput m_limitSwitch;
  private DigitalInput m_limitSwitch2;
  private TalonFxCHAOS m_extensionController;
  private DoubleSolenoid m_solenoidLeft;
  private DoubleSolenoid m_solenoidRight;
  private double m_downPositionCounts;
  private boolean m_seenBottom;

  /** Creates a new Climber. */
  public Climber() {
    m_extensionController = new TalonFxCHAOS(Constants.ClimberExtension, "Climber", "Extension");
    m_extensionController.setNeutralMode(NeutralMode.Brake);
    m_solenoidLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ClimberSolenoidLeftForward, Constants.ClimberSolenoidLeftReverse);
    m_solenoidRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ClimberSolenoidRightForward, Constants.ClimberSolenoidRightReverse);
    m_limitSwitch = new DigitalInput(Constants.ExtenderLimitSwitch);
    m_limitSwitch2 = new DigitalInput(Constants.ExtenderLimitSwitch2);
    m_extensionController.config_kP(0, 0.025);
    m_extensionController.config_kI(0, 0.0);
  }

  public boolean hasSeenBottom() {
    return m_seenBottom;
  }

  public boolean isCLimberAtBottom() {
    return !m_limitSwitch.get() || !m_limitSwitch2.get();
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
    m_solenoidLeft.set(Value.kForward);
    m_solenoidRight.set(Value.kForward);
  }

  public void MoveArmDown() {
    m_solenoidLeft.set(Value.kReverse);
    m_solenoidRight.set(Value.kReverse);
  }

  public void ReleaseArm() {
    m_solenoidLeft.set(Value.kOff);
    m_solenoidRight.set(Value.kOff);
    
  }
  

  public void ExtendToTop() {
    if(m_seenBottom) {
      m_extensionController.set(TalonFXControlMode.Position, m_downPositionCounts + 500000);
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
    SmartDashboard.putNumber("Climber/Position", m_extensionController.getSelectedSensorPosition());
    
  }
}
