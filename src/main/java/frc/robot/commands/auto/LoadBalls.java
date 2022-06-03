// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.chaos131.auto.ParsedCommand;
import com.chaos131.auto.commands.BaseAutoCommand;

import frc.robot.Robot;
import frc.robot.subsystems.Feeder;

/** Add your docs here. */
public class LoadBalls extends BaseAutoCommand {
    private double m_threshold_ms;
    private Feeder m_feeder;
    private double m_timeBothBallsSeenMs;
    private boolean m_containsTwoBalls = false;

    public LoadBalls(ParsedCommand parsedCommand, Feeder feeder) {
        super(parsedCommand);
        m_threshold_ms = AutoUtil.parseDouble(parsedCommand.getArgument("thresholdMs"), 0.0);
        m_feeder = feeder;
    }

    @Override
    protected void afterWorking() {

    }

    @Override
    protected void beforeWorking() {

    }

    @Override
    protected boolean isWorkDone() {
        return m_containsTwoBalls && (Robot.getCurrentTimeMs() >= m_timeBothBallsSeenMs + m_threshold_ms);
    }

    @Override
    protected void work() {
        if(m_feeder.IsBallAtMiddleFeeder() && m_feeder.IsBallAtTopFeeder() && !m_containsTwoBalls) {
            m_timeBothBallsSeenMs = Robot.getCurrentTimeMs();
            m_containsTwoBalls = true;

        }
    }

}
