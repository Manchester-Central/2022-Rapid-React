// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
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

}
