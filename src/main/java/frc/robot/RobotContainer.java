// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.auto.AutoBuilder;
import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClimberDefault;
import frc.robot.commands.DriveOverDistance;
import frc.robot.commands.DriveOverTime;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.FieldRelativeDrive;
import frc.robot.commands.IntakeDefault;
import frc.robot.commands.LauncherDefault;
import frc.robot.commands.FeederDefault;
import frc.robot.commands.RobotRelativeDrive;
import frc.robot.commands.SwerveModuleTest;
import frc.robot.commands.SwerveMotorTest;
import frc.robot.commands.ZeroNavX;
import frc.robot.commands.auto.AutoRobotRelativeDrive;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveModulePosition;

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

  private Gamepad m_driver = new Gamepad(0, "Driver", false);
  private Gamepad m_operator = new Gamepad(1, "Operator", false);
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);

  private SwerveDrive m_swerveDrive = new SwerveDrive();
  // private Climber m_climber = new Climber();
  private Camera m_camera = new Camera();
  private Launcher m_launcher = new Launcher();
  private Feeder m_feeder = new Feeder();
  //private Intake m_intake = new Intake();

  private AutoBuilder m_autoBuilder = new AutoBuilder();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_autoBuilder.registerCommand("robotRelativeDrive", (ParsedCommand pc) -> new AutoRobotRelativeDrive(pc, m_swerveDrive));
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
    // Default Commands
    //m_swerveDrive.setDefaultCommand(new FieldRelativeDrive(m_swerveDrive, m_driver));
    m_swerveDrive.setDefaultCommand(new RunCommand(() -> {m_swerveDrive.stop();}, m_swerveDrive));
    // m_swerveDrive.setDefaultCommand(new FieldRelativeDrive(m_swerveDrive, m_driver));
    // m_climber.setDefaultCommand(new ClimberDefault(m_climber));
    //m_intake.setDefaultCommand(new IntakeDefault(m_intake));
    m_launcher.setDefaultCommand(new LauncherDefault(m_launcher));
    m_feeder.setDefaultCommand(new FeederDefault(m_feeder));

    // Drive Commands
    Command driverRelativeDrive = new DriverRelativeDrive(m_swerveDrive, m_driver);
    Command robotRelativeDrive = new RobotRelativeDrive(m_swerveDrive, m_driver);
    m_driver.getButtonStart().whileHeld(new RunCommand(() -> {m_swerveDrive.ResetEncoders();}, m_swerveDrive));
    m_driver.getButtonA().whileHeld(driverRelativeDrive);
    m_driver.getButtonX().toggleWhenPressed(robotRelativeDrive);
    m_driver.getButtonY().whileActiveOnce(new SequentialCommandGroup(
        new DriveOverTime(m_swerveDrive, 0, 2, 0, 2000),
        new DriveOverTime(m_swerveDrive, 2, 0, 0, 2000),
        new DriveOverTime(m_swerveDrive, 0, -2, 3, 2000),
        new DriveOverTime(m_swerveDrive, -2, 0, 0, 2000)));
    /* m_driver.getButtonLB().whileHeld(new SwerveMotorTest(m_swerveDrive, m_driver, SwerveModulePosition.BackLeft));
    m_driver.getButtonRB().whileHeld(new SwerveMotorTest(m_swerveDrive, m_driver, SwerveModulePosition.BackRight));
    m_driver.getButtonLT().whileHeld(new SwerveMotorTest(m_swerveDrive, m_driver, SwerveModulePosition.FrontLeft));
    m_driver.getButtonRT().whileHeld(new SwerveMotorTest(m_swerveDrive, m_driver,  SwerveModulePosition.FrontRight)); */

    m_driver.getButtonLB().whileHeld(new SwerveModuleTest(m_swerveDrive, m_driver, SwerveModulePosition.BackLeft));
    m_driver.getButtonRB().whileHeld(new SwerveModuleTest(m_swerveDrive, m_driver, SwerveModulePosition.BackRight));
    m_driver.getButtonLT().whileHeld(new SwerveModuleTest(m_swerveDrive, m_driver, SwerveModulePosition.FrontLeft));
    m_driver.getButtonRT().whileHeld(new SwerveModuleTest(m_swerveDrive, m_driver, SwerveModulePosition.FrontRight));
    m_driver.getPOVNorth().whileActiveOnce(new ZeroNavX(0, m_swerveDrive));
    m_driver.getPOVEast().whileActiveOnce(new ZeroNavX(90, m_swerveDrive));
    m_driver.getPOVSouth().whileActiveOnce(new ZeroNavX(180, m_swerveDrive));
    m_driver.getPOVWest().whileActiveOnce(new ZeroNavX(270, m_swerveDrive));
    
    /*
     * m_driver.getButtonLB().whileActiveOnce(new DriveOverDistance(m_swerveDrive,
     * 2, -2, 1, 2));
     * m_driver.getButtonRB().whileActiveOnce(new DriveToPosition(m_swerveDrive, 0,
     * 0, 0));
     * m_driver.getButtonLT().whileActiveOnce(new DriveToPosition(m_swerveDrive,
     * 14.46, 6.60, -42.80));
     * m_driver.getButtonRT().whileActiveOnce(new DriveOverTime(m_swerveDrive, 0, 2,
     * 0, 2000));
     */

    // Operator Commands
    //m_operator.getButtonLB().whileHeld(new RunCommand(() -> m_climber.ManualExtend(-0.3), m_climber));
    //m_operator.getButtonLT().whileHeld(new RunCommand(() -> m_climber.ManualExtend(0.3), m_climber));
    //m_operator.getButtonRB().whileHeld(new RunCommand(() -> m_climber.MoveArmDown(), m_climber));
    //m_operator.getButtonRT().whileHeld(new RunCommand(() -> m_climber.MoveArmUp(), m_climber));
    m_operator.getButtonA().whileHeld(new RunCommand(() -> m_launcher.ManualLaunch(m_operator.getLeftY()), m_launcher));
    m_operator.getButtonB().whileHeld(new RunCommand(() -> m_feeder.ManualFeed(0.5, 0.5), m_feeder));
   /*m_operator.getButtonX().whileHeld(new RunCommand(() -> m_intake.ManualIntake(0.5), m_intake));
    m_operator.getButtonY().whileHeld(new RunCommand(() -> m_intake.ManualIntake(-0.5), m_intake));
    m_operator.getButtonSelect().whileHeld(new RunCommand(() -> m_intake.MoveIntakeDown(), m_intake));
    m_operator.getButtonStart().whileHeld(new RunCommand(() -> m_intake.MoveIntakeUp(), m_intake));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    return m_autoBuilder.createAutoCommand();
  }
}
