// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;

/** Add your docs here. */
public class LogManager {
    private boolean m_willLogShuffleBoard;
    private ShuffleboardTab m_shuffleboardTab;
    private List<String> m_headers = new ArrayList<String>();
    private Map<String, Supplier<String>> m_suppliers = new HashMap<String, Supplier<String>>();
    private LoggingThread m_loggingThread;

    public LogManager(boolean willLogShuffleBoard) {
        m_loggingThread = new LoggingThread();
        m_shuffleboardTab = Shuffleboard.getTab("Logging");
        m_willLogShuffleBoard = willLogShuffleBoard;
        addNumber("timeMs", () -> Robot.getCurrentTimeMs());
    }

    public void addBoolean(String title, BooleanSupplier valueSupplier) {
        m_headers.add(title);
        m_suppliers.put(title, () -> String.valueOf(valueSupplier.getAsBoolean()));
        if (m_willLogShuffleBoard) {
            m_shuffleboardTab.addBoolean(title, valueSupplier);
        }
    }

    public void addNumber(String title, DoubleSupplier valueSupplier) {
        m_headers.add(title);
        m_suppliers.put(title, () -> String.valueOf(valueSupplier.getAsDouble()));
        if (m_willLogShuffleBoard) {
            m_shuffleboardTab.addNumber(title, valueSupplier);
        }
    }

    public void addString(String title, Supplier<String> valueSupplier) {
        m_headers.add(title);
        m_suppliers.put(title, valueSupplier);
        if (m_willLogShuffleBoard) {
            m_shuffleboardTab.addString(title, valueSupplier);
        }
    }

    public void writeHeaders() {
        StringBuilder sb = new StringBuilder();
        for (String header : m_headers) {
            sb.append(header + ",");
        }
        m_loggingThread.enqueue(sb.toString());
    }

    public void update() {
        if (m_loggingThread.getState() == Thread.State.NEW) {
            if (DriverStation.isDSAttached()) {
                m_loggingThread.start();
            }
        } else {
            StringBuilder sb = new StringBuilder();
            for (String header : m_headers) {
                sb.append(m_suppliers.get(header).get() + ",");
            }
            m_loggingThread.enqueue(sb.toString());
        }
    }
}
