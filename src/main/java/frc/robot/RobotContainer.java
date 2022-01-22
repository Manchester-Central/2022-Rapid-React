// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveOverDistance;
import frc.robot.commands.DriveOverTime;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.FieldRelativeDrive;
import frc.robot.commands.RobotRelativeDrive;
import frc.robot.commands.SwerveModuleTest;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Gamepad m_driver = new Gamepad(0, "Driver");
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private SwerveDrive m_swerveDrive = new SwerveDrive();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_swerveDrive.setDefaultCommand(new FieldRelativeDrive(m_swerveDrive, m_driver));
    Command driverRelativeDrive = new DriverRelativeDrive(m_swerveDrive, m_driver);
    Command swerveModuleTest = new SwerveModuleTest(m_swerveDrive, m_driver);
    Command robotRelativeDrive = new RobotRelativeDrive(m_swerveDrive, m_driver);
    m_driver.getButtonA().whileHeld(driverRelativeDrive);
    m_driver.getButtonB().whileHeld(swerveModuleTest);
    m_driver.getButtonX().whileHeld(robotRelativeDrive);
    m_driver.getButtonY().whileActiveOnce(new SequentialCommandGroup(
      new DriveOverTime(m_swerveDrive, 0, 2, 0, 2000),
      new DriveOverTime(m_swerveDrive, 2, 0, 0, 2000),
      new DriveOverTime(m_swerveDrive, 0, -2, 3, 2000),
      new DriveOverTime(m_swerveDrive, -2, 0, 0, 2000)
    ));
    m_driver.getButtonLB().whileActiveOnce(new DriveOverDistance(m_swerveDrive, 2, -2, 1, 2));
    m_driver.getButtonRB().whileActiveOnce(new DriveToPosition(m_swerveDrive, 0, 0));
    m_driver.getButtonLT().whileActiveOnce(new DriveOverTime(m_swerveDrive, -2, 2, 0, 1000));
    m_driver.getButtonRT().whileActiveOnce(new DriveOverTime(m_swerveDrive, 0, 2, 0, 2000));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
    return null;
  }
}
