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

    private double distance;
    private double speed;

    public TableData(double dist, double spd) {

        distance = dist;
        speed = spd;

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
        return distance;
    }

    public double getSpeed() {
        return speed;
    }

    @Override
    public String toString() {
        return "distance=" + distance + ", speed = " + speed;
    }
}
