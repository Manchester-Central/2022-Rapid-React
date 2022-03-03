// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.concurrent.ConcurrentLinkedQueue;

import edu.wpi.first.wpilibj.RobotBase;

/** Add your docs here. */
public class LoggingThread extends Thread {
    private static final double FLUSH_THRESHOLD = 200;
    private Date m_timestamp;
    private FileWriter m_fileWriter;
    private PrintWriter m_PrintWriter;
    private ConcurrentLinkedQueue<String> m_q;

    public LoggingThread() {
        timestampNow();
        m_q = new ConcurrentLinkedQueue<String>();

    }

    public void enqueue(String s) {
        m_q.add(s);
    }

    public void timestampNow() {
        m_timestamp = Calendar.getInstance().getTime();
    }

    public void run() {
        int unflushed = 0;
        timestampNow();
        try {
            var simpleDateFormat = new SimpleDateFormat("yyyy-MM-dd'T'HH-mm-ss.SSSZ");
            var dateString = simpleDateFormat.format(m_timestamp);
            var filePath = "logs/log-" + dateString + ".csv";
            if (RobotBase.isReal()) {
                // Save data to USB
                filePath = "/U/" + filePath;
            }
            Path pathToFile = Paths.get(filePath);
            Files.createDirectories(pathToFile.getParent());
            m_fileWriter = new FileWriter(filePath);
            m_PrintWriter = new PrintWriter(m_fileWriter);
        } catch (IOException e) {
            System.err.println("failed to load log file");
            e.printStackTrace();
            return;
        }

        boolean running = true;
        while (running) {
            try {

                String s = m_q.poll();
                while (s != null) {
                    recordRow(s);
                    unflushed += 1;
                    if (unflushed > FLUSH_THRESHOLD) {
                        flush();
                        unflushed = 0;
                    }
                    s = m_q.poll();
                }
                Thread.sleep(100);
            } catch (InterruptedException e) {
                System.err.println("Logging thread InterruptedException");
                running = false; // TODO IS THIS RIGHT?
            }
        }

    }

    private void recordRow(String s) {
        m_PrintWriter.println(s);
        m_PrintWriter.flush();
    }

    private void flush() {
        // TODO ?
    }
}
