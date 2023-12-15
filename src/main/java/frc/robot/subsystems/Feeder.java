// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Feeder extends SubsystemBase {
  private WPI_TalonFX m_feederUpper;
  private WPI_TalonFX m_feederLower;

public class OneshotCommand extends CommandBase{

  private DigitalInput m_sensorLower; // Lower = 2//
  private DigitalInput m_sensorUpper; // Upper = 1//
  public OneshotCommand() {
  m_sensorLower = new DigitalInput(2);
  m_sensorUpper = new DigitalInput(1);
  }

  /** The main body of a command. Called repeatedly while the command is scheduled. */
  public void execute() {}

  /**
   * The action to take when the command ends. Called when either the command finishes normally, or
   * when it interrupted/canceled.
   *
   * <p>Do not schedule commands here that share requirements with this command. Use {@link
   * #andThen(Command...)} instead.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  public void end(boolean interrupted) {}

  /**
   * Whether the command has finished. Once a command finishes, the scheduler will call its end()
   * method and un-schedule it.
   *
   * @return whether the command has finished.
   */
  public boolean isFinished() {
   if ( m_sensorUpper.get()){
   m_feederUpper.set(0);
   return true;
   } else {
    return false;
   }
  
  }

}




  /** Creates a new feeder. */
  public Feeder() {
    m_feederLower = new WPI_TalonFX(Constants.FeederConstants.FeederLowerCanID);
    m_feederUpper = new WPI_TalonFX(Constants.FeederConstants.FeederUpperCanID);
    setDefaultCommand(new OneshotCommand() {
      
    });
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
