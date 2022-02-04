// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

/** Add your docs here. */
public class SwerveDriveModule {
    private Translation2d m_location;
    private Field2d m_field = new Field2d();
    private double m_targetVelocity = 0;
    private double m_targetAngle = 0;
    private TalonFX m_velocityController;
    private TalonFX m_angleController;
    private String m_name;
    private double m_angleOffset;

    public SwerveDriveModule(double x, double y, double angleOffset, String name, int velocityControllerPort,
            int angleControllerPort) {
        m_location = new Translation2d(x, y);
        SmartDashboard.putData(name, m_field);
        m_velocityController = new TalonFX(velocityControllerPort);
        m_angleController = new TalonFX(angleControllerPort);
        m_name = name;
        m_angleOffset = angleOffset;
        Robot.LogManager.addNumber(m_name + "/targetVelocityMPS", () -> m_targetVelocity);
        Robot.LogManager.addNumber(m_name + "/targetAngleDegrees", () -> m_targetAngle);
        Robot.LogManager.addNumber(m_name + "/actualVelocityMPS", () -> getCurrentVelocityMPS());
        Robot.LogManager.addNumber(m_name + "/actualAngleDegrees", () -> getCurrentAngleDegrees());

    }

    public void updatePosition(Pose2d robotPose) {
        Pose2d modulePose = robotPose.transformBy(new Transform2d(m_location, Rotation2d.fromDegrees(m_targetAngle)));
        m_field.setRobotPose(modulePose);
    }

    public double getCurrentVelocityMPS() {
        if (RobotBase.isReal()) {
            return FalconVelocityToMPS(m_velocityController.getSelectedSensorVelocity());
        }
        return m_targetVelocity;
    }

    public double getCurrentAngleDegrees() {
        if (RobotBase.isReal()) {
            return FalconAngleToDegrees(m_angleController.getSelectedSensorPosition()) - m_angleOffset;
        }
        return m_targetAngle;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getCurrentVelocityMPS(), Rotation2d.fromDegrees(getCurrentAngleDegrees()));
    }

    public void setTargetState(SwerveModuleState targetState) {
        m_targetVelocity = targetState.speedMetersPerSecond;
        m_targetAngle = targetState.angle.getDegrees();
        m_velocityController.set(TalonFXControlMode.Velocity, MPSToFalconVelocity(m_targetVelocity));
        m_angleController.set(TalonFXControlMode.Position, DegreesToFalconAngle(m_targetAngle + m_angleOffset));
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
        // Calculate ratio of the full rotation of the wheel

        var wheelRotations = degrees / 360;

        // Convert to rotations of the motor

        var motorRotations = Constants.SwerveModuleAngleGearRatio * wheelRotations;

        // Convert to # of counts

        return motorRotations * Constants.TalonCountsPerRevolution;

    }

    private double FalconAngleToDegrees(double FalconAngle) {
        var motorRotations = FalconAngle / Constants.TalonCountsPerRevolution;
        var wheelRotations = motorRotations / Constants.SwerveModuleAngleGearRatio;
        return wheelRotations * 360;
    }

}