// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Feeder extends SubsystemBase {
  private WPI_TalonFX m_feederUpper;
  private WPI_TalonFX m_feederLower;

  /** Creates a new feeder. */
  public Feeder() {
    m_feederLower = new WPI_TalonFX(Constants.FeederConstants.FeederLowerCanID);
    m_feederUpper = new WPI_TalonFX(Constants.FeederConstants.FeederUpperCanID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void stop() {
    m_feederLower.stopMotor();
    m_feederUpper.stopMotor();
  }

  public void setSpeed(double speed) {
    m_feederLower.set(speed);
    m_feederUpper.set(speed);

  }
  
}
