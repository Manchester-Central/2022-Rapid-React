// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class LogManager {
    private boolean m_willLogShuffleBoard;
    private ShuffleboardTab m_shuffleboardTab;
    private FileWriter m_fileWriter;
    private PrintWriter m_PrintWriter;
    private List<String> m_headers = new ArrayList<String>();
    private Map<String, Supplier<String>> m_suppliers = new HashMap<String, Supplier<String>>();

    public LogManager(boolean willLogShuffleBoard) {
        
        m_shuffleboardTab = Shuffleboard.getTab("Logging");
        m_willLogShuffleBoard = willLogShuffleBoard;
        try {
            Calendar rightNow = Calendar.getInstance();
            var simpleDateFormat = new SimpleDateFormat("yyyy-MM-dd'T'HH-mm-ss.SSSZ");
            var dateString = simpleDateFormat.format(rightNow.getTime()); 
            m_fileWriter = new FileWriter("log-" +dateString+".csv");
            m_PrintWriter = new PrintWriter(m_fileWriter);
        } catch (IOException e) {
            System.err.println("failed to load log file");
            e.printStackTrace();
        }
        addNumber("timeMs", () -> RobotController.getFPGATime() / 1000);
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
        if (m_PrintWriter == null) {
            return;
        }
        try {
            for (String header : m_headers) {
                m_PrintWriter.print(header + ",");
            }
            m_PrintWriter.println();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    public void writeLine() {
        if (m_PrintWriter == null) {
            return;
        }
        try {
            for (String header : m_headers) {
                m_PrintWriter.print(m_suppliers.get(header).get() + ",");
            }
            m_PrintWriter.println();
            m_PrintWriter.flush();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
