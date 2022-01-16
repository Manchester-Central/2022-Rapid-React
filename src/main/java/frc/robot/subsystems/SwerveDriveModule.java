// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveDriveModule {
    private Translation2d m_location;
    private Field2d m_field = new Field2d();
    private double m_velocity = 0;
    private double m_angle = 0;

    public SwerveDriveModule(double x, double y, String name) {
        m_location = new Translation2d(x, y);
        SmartDashboard.putData(name, m_field);
    }

    public void updatePosition(Pose2d robotPose) {
        Pose2d modulePose = robotPose.transformBy(new Transform2d(m_location, Rotation2d.fromDegrees(m_angle)));
        m_field.setRobotPose(modulePose);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_velocity, Rotation2d.fromDegrees(m_angle));
    }

    public void setTargetState(SwerveModuleState targetState) {
        m_velocity = targetState.speedMetersPerSecond;
        m_angle = targetState.angle.getDegrees();
    }

    public Translation2d getLocation() {
        return m_location;
    }
}
