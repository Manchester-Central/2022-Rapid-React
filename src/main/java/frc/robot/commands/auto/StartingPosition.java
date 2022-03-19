// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.auto.commands.BaseAutoCommand;

import frc.robot.subsystems.SwerveDrive;

/** Add your docs here. */
public class StartingPosition extends BaseAutoCommand {
    private SwerveDrive m_SwerveDrive;
    private double m_x, m_y, m_angle;

    public StartingPosition(ParsedCommand parsedCommand, SwerveDrive swerveDrive) {
        super(parsedCommand);
        m_SwerveDrive = swerveDrive;
        m_x = AutoUtil.parseDouble(parsedCommand.getArgument("x"), 0.0);
        m_y = -AutoUtil.parseDouble(parsedCommand.getArgument("y"), 0.0); // negative is workaround for coordinate issue
        m_angle = AutoUtil.parseDouble(parsedCommand.getArgument("angle"), 0.0);
    }

    @Override
    protected void afterWorking() {

    }

    @Override
    protected void beforeWorking() {

    }

    @Override
    protected boolean isWorkDone() {
        return true;
    }

    @Override
    protected void work() {
        m_SwerveDrive.updateOdometry(m_x, m_y, m_angle);
    }

}
