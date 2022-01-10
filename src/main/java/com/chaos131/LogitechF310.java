/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.chaos131;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * Add your docs here.
 */
public class LogitechF310 extends Joystick {
    public LogitechF310(int port) {
        super(port);
        xButton = new JoystickButton(this, LEFT_X);
        aButton = new JoystickButton(this, DOWN_A);
        bButton = new JoystickButton(this, RIGHT_B);
        yButton = new JoystickButton(this, UP_Y);
        startButton = new JoystickButton(this, START);
        selectButton = new JoystickButton(this, SELECT);
        leftBumper = new JoystickButton(this, LEFT_BUMPER);
        rightBumper = new JoystickButton(this, RIGHT_BUMPER);
        leftTrigger = new JoystickButton(this, LEFT_TRIGGER);
        rightTrigger = new JoystickButton(this, RIGHT_TRIGGER);
        leftJoystick = new JoystickButton(this, LEFT_JOYSTICK);
        rightJoystick = new JoystickButton(this, RIGHT_JOYSTICK);
        dPadUp = new POVButton(this, DPAD_UP);
        dPadRight = new POVButton(this, DPAD_RIGHT);
        dPadDown = new POVButton(this, DPAD_DOWN);
        dPadLeft = new POVButton(this, DPAD_LEFT);
    }

    private static final int LEFT_X = 1;
    private static final int DOWN_A = 2;
    private static final int RIGHT_B = 3;
    private static final int UP_Y = 4;

    private static final int LEFT_BUMPER = 5;
    private static final int RIGHT_BUMPER = 6;
    private static final int LEFT_TRIGGER = 7;
    private static final int RIGHT_TRIGGER = 8;

    private static final int SELECT = 9;
    private static final int START = 10;
    private static final int LEFT_JOYSTICK = 11;
    private static final int RIGHT_JOYSTICK = 12;

    private static final int DPAD_UP = 0;
    private static final int DPAD_RIGHT = 90;
    private static final int DPAD_DOWN = 180;
    private static final int DPAD_LEFT = 270;

    public JoystickButton aButton;
    public JoystickButton bButton;
    public JoystickButton xButton;
    public JoystickButton yButton;

    public JoystickButton startButton;
    public JoystickButton selectButton;

    public JoystickButton leftBumper;
    public JoystickButton rightBumper;
    public JoystickButton leftTrigger;
    public JoystickButton rightTrigger;

    public JoystickButton leftJoystick;
    public JoystickButton rightJoystick;

    public POVButton dPadUp;
    public POVButton dPadRight;
    public POVButton dPadDown;
    public POVButton dPadLeft;

    public double getLeftX() {
        return getRawAxis(0);
    }

    public double getLeftY() {
        return -getRawAxis(1);
    }

    public double getRightX() {
        return getRawAxis(2);
    }

    public double getRightY() {
        return -getRawAxis(3);
    }

    private double calculateAngle(double x, double y) {
        return -Math.toDegrees(Math.atan2(-x, y));
    }

    public double getRightJoystickAngle() {
        return calculateAngle(getRightX(), getRightY());
    }

    public double getLeftJoystickAngle() {
        return calculateAngle(getLeftX(), getLeftY());
    }
}
