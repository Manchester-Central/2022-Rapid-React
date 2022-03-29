/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Comparator;

/**
 * Add your docs here.
 */
public class TableData {

    private double m_distance;
    private double m_speed;
    private boolean m_hoodUp;

    public TableData(double distance, double speed, boolean hoodUp) {

        m_distance = distance;
        m_speed = speed;
        m_hoodUp = hoodUp;

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

    @Override
    public String toString() {
        return "distance=" + m_distance + ", speed = " + m_speed;
    }
}
