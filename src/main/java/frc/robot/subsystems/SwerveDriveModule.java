// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveDriveModule {
    private Translation2d m_location;
    private Field2d m_field = new Field2d();
    private double m_velocity = 0;
    private double m_angle = 0;
    private TalonFX m_velocityController;
    private TalonFX m_angleController;

    public SwerveDriveModule(double x, double y, String name, int velocityControllerPort, int angleControllerPort) {
        m_location = new Translation2d(x, y);
        SmartDashboard.putData(name, m_field);
        m_velocityController = new TalonFX(velocityControllerPort);
        m_angleController = new TalonFX(angleControllerPort);
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
        // TODO: figure out velocity and angle in terms of encoder ticks
        m_velocityController.set(TalonFXControlMode.Velocity, MPSToFalconVelocity(m_velocity));
        m_angleController.set(TalonFXControlMode.Position, m_angle);
    }

    public Translation2d getLocation() {
        return m_location;
    }

    private double MPSToFalconVelocity(double mps) {
        // Distance conversion from meters to encoder counts
        
        var rps = mps / Constants.DriveWheelCircumferenceMeters;
        var countsPerSecond = rps * Constants.SwerveModuleVelocityGearRatio * Constants.TalonCountsPerRevolution;

        // Time conversion from seconds to 100 milliseconds

        return countsPerSecond / 10;

    }

    private double FalconVelocityToMPS(double FalconVelocity) {
        var countsPerSecond = FalconVelocity * 10;
        var rps = countsPerSecond / (Constants.SwerveModuleVelocityGearRatio * Constants.TalonCountsPerRevolution);
        var mps = rps * Constants.DriveWheelCircumferenceMeters;
        return mps;
    }

    private double DegreesToFalconAngle(double degrees) {
        //Calculate ratio of the full rotation of the wheel

        var wheelRotations = degrees / 360;

        //Convert to rotations of the motor

        var motorRotations = Constants.SwerveModuleAngleGearRatio * wheelRotations;

        //Convert to # of counts

        return motorRotations * Constants.TalonCountsPerRevolution;

    }
}
