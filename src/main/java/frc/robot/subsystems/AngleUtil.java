// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class AngleUtil {
    
    public static double closestTarget(double currentAngle, double targetAngle) {
        targetAngle = targetAngle + (Math.floor(currentAngle / 360) * 360);
        double otherAngle = targetAngle + 360;
        if (targetAngle > currentAngle) {
            otherAngle = targetAngle - 360;
        }
        double distanceNegative = Math.abs(targetAngle - currentAngle);
        double distancePositive = Math.abs(otherAngle - currentAngle);
        if (distanceNegative < distancePositive) {
            return targetAngle;
        } else {
            return otherAngle;
        }
    }
}
