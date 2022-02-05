// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static int RobotUpdate_hz = 20;

    // CAN device numbers
    public final static int SwerveFrontLeftVelocity = 1;
    public final static int SwerveFrontLeftAngle = 2;
    public final static int SwerveFrontRightVelocity = 3;
    public final static int SwerveFrontRightAngle = 4;
    public final static int SwerveBackLeftVelocity = 5;
    public final static int SwerveBackLeftAngle = 6;
    public final static int SwerveBackRightVelocity = 7;
    public final static int SwerveBackRightAngle = 8;
    public final static int ClimberExtension = 9;
    public final static int LauncherA = 10;
    public final static int LauncherB = 11;
    public final static int LoaderTop = 12;
    public final static int LoaderBottom = 13;
    public final static int Intake = 14;
    
    // Drivetrain Properties
    public final static double DriveWheelWidthMeters = 0.1016; // 0.1016 meters = 4 inches, 1 rev of motor = 2048 tics
    public final static double DriveWheelCircumferenceMeters = DriveWheelWidthMeters * Math.PI;
    public final static double SwerveModuleVelocityGearRatio = 8.14; // Assuming the Standard Gear Ratio; Same link as
                                                                     // below
    public final static double SwerveModuleAngleGearRatio = 12.8; // https://www.swervedrivespecialties.com/products/mk4-swerve-module
    public final static double TalonCountsPerRevolution = 2048;

    // Pneumatic Connections
    public final static int SolenoidForward = 1;
    public final static int SolenoidReverse = 2;
}
