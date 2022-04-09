/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Comparator;

import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class TableData {

    private double m_distance;
    private double m_speed;
    private boolean m_hoodUp;
    private double m_launcherTolerance;
    private double m_feederSpeed;

    public TableData(double distance, double speed, boolean hoodUp, double launcherTolerance, double feederSpeed) {

        m_distance = distance;
        m_speed = speed;
        m_hoodUp = hoodUp;
        m_launcherTolerance = launcherTolerance;
        m_feederSpeed = feederSpeed;

    }

    public static TableData FromCSV(String[] args) throws Exception {
        var distance = getValueFromStringArray(args, 0, true, "distance");
        var speed = getValueFromStringArray(args, 1, true, "speed");
        var hoodUp = getValueFromStringArray(args, 2, false, "hoodUp");
        var launcherTolerance = getValueFromStringArray(args, 3, false, "launcherTolerance");
        var feederSpeed = getValueFromStringArray(args, 4, false, "feederSpeed");

        return new TableData(
            Double.parseDouble(distance),
            Double.parseDouble(speed),
            isValueSet(hoodUp) ? Boolean.parseBoolean(hoodUp) : false,
            isValueSet(launcherTolerance) ? Double.parseDouble(launcherTolerance) : Constants.DefaultLauncherTolerance,
            isValueSet(feederSpeed) ? Double.parseDouble(feederSpeed) : Constants.DefaultFeederLaunchSpeed
        );
    }

    private static String getValueFromStringArray(String[] args, int index, boolean isRequired, String columnName) throws Exception {
        String value = null;
        if(args.length >= index + 1) {
            value = args[index];
        }

        if(!isValueSet(value) && isRequired) {
            throw new Exception("Missing required value from CSV: " + columnName);
        }
        return value;
    }

    private static boolean isValueSet(String value) {
        return value != null && !value.isBlank();
    }

    public static Comparator<TableData> getComparator() {
        return new Comparator<TableData>() {
            @Override
            public int compare(TableData arg0, TableData arg1) {
                if (arg1.getDistance() == arg0.getDistance()){
                    return 0;
                }
                return arg1.getDistance() < arg0.getDistance() ? 1 : -1;
            }
        };
    }

    public double getDistance() {
        return m_distance;
    }

    public double getSpeed() {
        return m_speed;
    }
    
    public boolean getHoodUp() {
        return m_hoodUp;
    }
    
    public double getLauncherTolerance() {
        return m_launcherTolerance;
    }
    
    public double getFeederSpeed() {
        return m_feederSpeed;
    }

    @Override
    public String toString() {
        return "distance=" + m_distance + ", speed = " + m_speed;
    }
}
