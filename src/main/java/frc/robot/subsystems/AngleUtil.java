// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
            var aimXAngle = -camera.getXAngle();
            var aimCurrentAngle = currentAngle;
            var targetAngle = Rotation2d.fromDegrees(aimXAngle).plus(aimCurrentAngle).getDegrees();
            targetAngle = AngleUtil.clampAngle(targetAngle);
            SmartDashboard.putNumber("aim/xAdjust", aimXAngle);
            SmartDashboard.putNumber("aim/currentAngle", aimCurrentAngle.getDegrees());
            SmartDashboard.putNumber("aim/targetAngle", targetAngle);
            return Rotation2d.fromDegrees(targetAngle);
        }
        /*
         * var relativeLocation = currentPose.relativeTo(new
         * Pose2d(Constants.GoalLocation, Rotation2d.fromDegrees(0)));
         * var angle = new Rotation2d(relativeLocation.getX(), relativeLocation.getY());
         * angle = angle.rotateBy(Rotation2d.fromDegrees(180)).times(-1);
         * return angle;
         */
        return currentAngle;
    }

    public static double clampAngle(double angle) {
        return Math.IEEEremainder(angle, 360);
    }

}
