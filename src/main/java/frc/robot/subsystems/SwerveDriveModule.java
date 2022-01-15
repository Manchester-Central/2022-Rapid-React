// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveDriveModule {
    Translation2d m_location;
    private Field2d m_field = new Field2d();
    public SwerveDriveModule(double x, double y, String name) {
        m_location = new Translation2d (x, y);
        SmartDashboard.putData(name, m_field);
    }
}
