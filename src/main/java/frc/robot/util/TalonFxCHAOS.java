// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class TalonFxCHAOS extends TalonFX {

    private TalonFXControlMode m_lastMode = TalonFXControlMode.Disabled;
    private double m_lastValue = Double.NaN;;

    public TalonFxCHAOS(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void set(TalonFXControlMode mode, double value) {
        if(mode != m_lastMode || value != m_lastValue) {
            super.set(mode, value);
        }
        m_lastMode = mode;
        m_lastValue = value;
    }
    
}
