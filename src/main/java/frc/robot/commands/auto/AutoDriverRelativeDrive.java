// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.auto.commands.BaseAutoCommand;

import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class AutoDriverRelativeDrive extends BaseAutoCommand {
    SwerveDrive m_drive;
    double m_sidewaysSpeed;
    double m_forwardSpeed;
    double m_thetaSpeed;

    public AutoDriverRelativeDrive(ParsedCommand parsedCommand, SwerveDrive swerveDrive) {
        super(parsedCommand);
        m_drive = swerveDrive;
        m_sidewaysSpeed = AutoUtil.parseDouble(parsedCommand.getArgument("sideSpeed"), 0.0);
        m_forwardSpeed = AutoUtil.parseDouble(parsedCommand.getArgument("forwardSpeed"), 0.0);
        m_thetaSpeed = AutoUtil.parseDouble(parsedCommand.getArgument("thetaSpeed"), 0.0);
        addRequirements(swerveDrive);
    }

    @Override
    protected boolean isWorkDone() {
        return false;
    }

    @Override
    protected void beforeWorking() {
    }

    @Override
    protected void work() {
        m_drive.moveDriverRelative(m_sidewaysSpeed, m_forwardSpeed, m_thetaSpeed);
    }

    @Override
    protected void afterWorking() {
        m_drive.moveRobotRelative(0, 0, 0);
    }

}
