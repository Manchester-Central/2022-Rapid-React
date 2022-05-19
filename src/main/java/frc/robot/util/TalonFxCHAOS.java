// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Robot;

/** Add your docs here. */
public class TalonFxCHAOS extends TalonFX {
    public static boolean EnableStickyMode = true;

    private static ArrayList<TalonFxCHAOS> AllTalons = new ArrayList<TalonFxCHAOS>();
    public static void ResetStickiness() {
        for(var talon: AllTalons) {
            talon.m_lastMode = TalonFXControlMode.Disabled;
            talon.m_lastValue = Double.NaN;
        }
    }

    private TalonFXControlMode m_lastMode = TalonFXControlMode.Disabled;
    private double m_lastValue = Double.NaN;
    private String m_name;
    private String m_subsystem;

    public TalonFxCHAOS(int deviceNumber, String subsystem, String name) {
        super(deviceNumber);
        m_subsystem = subsystem;
        m_name = name;
        this.configFactoryDefault();
        AllTalons.add(this);
        Robot.LogManager.addNumber(getLogColumn("rawVelocity"), () -> this.getSelectedSensorVelocity());
        Robot.LogManager.addNumber(getLogColumn("rawPosition"), () -> this.getSelectedSensorVelocity());
        Robot.LogManager.addNumber(getLogColumn("temp"), () -> this.getTemperature());
        Robot.LogManager.addNumber(getLogColumn("supplyCurrent"), () -> this.getSupplyCurrent());
        Robot.LogManager.addString(getLogColumn("lastError"), () -> this.getLastError().toString());
    }

    private String getLogColumn(String column) {
        return m_subsystem + "-" + m_name + "-" + column;
    }

    @Override
    public void set(TalonFXControlMode mode, double value) {
        if(!EnableStickyMode) {
            super.set(mode, value);
            return;
        }
        if(mode != m_lastMode || value != m_lastValue) {
            super.set(mode, value);
        }
        m_lastMode = mode;
        m_lastValue = value;
    }
    
}
