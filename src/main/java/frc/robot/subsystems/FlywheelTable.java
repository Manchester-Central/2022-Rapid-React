/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.turret;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Add your docs here.
 */
public class FlywheelTable {

    String row;
    BufferedReader csvReader;
    final String PATH = "flywheelTable.csv";
    String[] data;
    double[] doubleData;
    Path realPath = Filesystem.getDeployDirectory().toPath().resolve(PATH);

    // holds an ArrayList with a key (distance) as reference to [key]
    ArrayList<TableData> flyTable = new ArrayList<TableData>();

    public FlywheelTable() {
        readCSV(PATH);
    }

    // parses data from .csv into doubles for addData()
    public void readCSV(String path) { // change return type
        try {
            // System.out.println(realPath.toString());
            csvReader = new BufferedReader(new FileReader(realPath.toString()));
        } catch (FileNotFoundException ie) {
            ie.printStackTrace();
        }

        try {
            csvReader.readLine(); // skips 1st line
            while ((row = csvReader.readLine()) != null) {
                // System.out.println(row);
                data = row.split(",");

                if (data.length == 3) {

                    // converts String array to double array
                    doubleData = Arrays.stream(data).mapToDouble(Double::parseDouble).toArray();

                    double distance = doubleData[0];
                    double speed = doubleData[1];
                    double angle = doubleData[2];

                    addData(new TableData(distance, speed, angle));

                } else {
                    System.out.println("ERROR: Flywheel Table data less than 3 values at " + row);
                }
            }

            flyTable.sort(TableData.getComparator());
            for (TableData data : flyTable) {
                System.out.println(data.getDistance() + " " + data.getSpeed() + " " + data.getAngle());
            }

        } catch (IOException ie) {
            ie.printStackTrace();
        }
    }

    // takes raw data, adds to data structure
    private void addData(TableData data) {
        // System.out.println(data.getDistance() + " " + data.getSpeed() + " " + data.getAngle());
        flyTable.add(data);
    }

    private TableData getTableData(int index) {
        return flyTable.get(index);
    }

    private double getDistance(int index) {
        return getTableData(index).getDistance();
    }

    private double getSpeed(int index) {
        return getTableData(index).getSpeed();
    }

    private double getAngle(int index) {
        return getTableData(index).getAngle();
    }

    private int findIndex(double distance) {
        for (int i = 0; i < flyTable.size(); i++) {
            if (distance < getDistance(i)) {
                return i;
            }
        }
        return flyTable.size() - 1;
    }

    private double getInterpolatedValue(double x1, double x2, double y1, double y2, double distance) {
        double slope = (y2 - y1) / (x2 - x1);
        double intercept = y1 - (slope * x1);

        return (slope * distance) + intercept;
    }

    public TableData getIdealTarget(double distance) {
        int initialIndex = findIndex(distance);
        int topIndex = initialIndex == 0 ? 1 : findIndex(distance);
        int botIndex = topIndex - 1;

        double idealSpeed = getInterpolatedValue(getDistance(topIndex), getDistance(botIndex), getSpeed(topIndex), getSpeed(botIndex), distance);
        double idealAngle = getInterpolatedValue(getDistance(topIndex), getDistance(botIndex), getAngle(topIndex), getAngle(botIndex), distance);

        return new TableData(distance, idealSpeed, idealAngle);
    }
}
