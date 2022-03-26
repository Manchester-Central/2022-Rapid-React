// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

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

public static Rotation2d GetEstimatedAngleToGoal(Camera camera, Pose2d currentPose, Rotation2d currentAngle) {
    if (camera.hasTarget()) {
        return Rotation2d.fromDegrees(camera.getXAngle()).plus(currentAngle);
    }
    var relativeLocation = currentPose.relativeTo(new Pose2d(Constants.GoalLocation, Rotation2d.fromDegrees(0)));
    var angle = new Rotation2d(relativeLocation.getX(), relativeLocation.getY());
    angle = angle.rotateBy(Rotation2d.fromDegrees(180)).times(-1);
    return angle;
}

}
