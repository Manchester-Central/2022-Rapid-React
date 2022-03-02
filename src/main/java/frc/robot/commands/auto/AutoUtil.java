// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

/** Add your docs here. */
public class AutoUtil {
    public static double parseDouble(String valueToParse, double defaultValue) {
        if(valueToParse == null) {
            return defaultValue;
        }
        return Double.parseDouble(valueToParse);
    }
}
