// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class LogManager {
    private boolean m_willLogShuffleBoard;
    private ShuffleboardTab m_shuffleboardTab;

    public LogManager(boolean willLogShuffleBoard) {
        m_shuffleboardTab = Shuffleboard.getTab("Logging");
        m_willLogShuffleBoard = willLogShuffleBoard;
    }

    public void addBoolean(String title, BooleanSupplier valueSupplier) {
        if (m_willLogShuffleBoard) {
            m_shuffleboardTab.addBoolean(title, valueSupplier);
        }
    }

    public void addNumber(String title, DoubleSupplier valueSupplier) {
        if (m_willLogShuffleBoard) {
            m_shuffleboardTab.addNumber(title, valueSupplier);
        }
    }

    public void addString(String title, Supplier<String> valueSupplier) {
        if (m_willLogShuffleBoard) {
            m_shuffleboardTab.addString(title, valueSupplier);
        }
    }
}
