// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.gamepads.Gamepad;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.CameraDefault;
import frc.robot.commands.DashboardSpeedLauncherShoot;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.FeederDefault;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefault;
import frc.robot.commands.IntakeWithOnlyFeeder;
import frc.robot.commands.LauncherDefault;
import frc.robot.commands.SetSpeedLauncherShoot;
import frc.robot.commands.ZeroNavX;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.FeederMode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Gamepad m_demoController = new Gamepad(0, "Demo", false);

  private Camera m_camera = new Camera();
  private Launcher m_launcher = new Launcher();
  private Feeder m_feeder = new Feeder();
  private Intake m_intake = new Intake();
  private SwerveDrive m_swerveDrive = new SwerveDrive();

  public PowerDistribution m_pdp = new PowerDistribution();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_intake.setDefaultCommand(new IntakeDefault(m_intake));
    m_launcher.setDefaultCommand(new LauncherDefault(m_launcher));
    m_feeder.setDefaultCommand(new FeederDefault(m_feeder));
    m_camera.setDefaultCommand(new CameraDefault(m_camera));
    m_swerveDrive.setDefaultCommand(new DriverRelativeDrive(m_swerveDrive, m_demoController));

    configureMatchCommands();

  }

  private void configureMatchCommands() {

    // Intake controls
    m_demoController.getButtonLB().whileHeld(new IntakeWithOnlyFeeder(m_feeder));
    m_demoController.getButtonLT().whileHeld(new IntakeCommand(m_feeder, m_intake));

    // Static launch commands
    m_demoController.getButtonA() // Low
        .whileHeld(new SetSpeedLauncherShoot(m_launcher, m_feeder, 2300, FeederMode.LAUNCH_LOW_BUMPER,
            Constants.DefaultLauncherTolerance));
    m_demoController.getButtonB() // Medium
        .whileHeld(new SetSpeedLauncherShoot(m_launcher, m_feeder, 2300, FeederMode.LAUNCH_LOW_BUMPER,
            Constants.DefaultLauncherTolerance));
    m_demoController.getButtonX() // Medium
        .whileHeld(new SetSpeedLauncherShoot(m_launcher, m_feeder, 2300, FeederMode.LAUNCH_LOW_BUMPER,
            Constants.DefaultLauncherTolerance));
    m_demoController.getButtonY() // High
        .whileHeld(new SetSpeedLauncherShoot(m_launcher, m_feeder, 2300, FeederMode.LAUNCH_LOW_BUMPER,
            Constants.DefaultLauncherTolerance));
    
    // Dashboard launch
    m_demoController.getButtonRT().whileHeld(new DashboardSpeedLauncherShoot(m_launcher, m_feeder));
        
    // Hood controls
    m_demoController.getButtonStart().whileHeld(new RunCommand(() -> m_launcher.MoveHoodUp(), m_launcher));
    m_demoController.getButtonSelect().whileHeld(new RunCommand(() -> m_launcher.MoveHoodDown(), m_launcher));

    // Gyro controls
    m_demoController.getPOVNorth().whileActiveOnce(new ZeroNavX(0, m_swerveDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
