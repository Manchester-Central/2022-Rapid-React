// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.auto.AutoBuilder;
import com.chaos131.auto.ParsedCommand;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimToGoal;
import frc.robot.commands.CameraDefault;
import frc.robot.commands.EnableSlowDriverSpeed;
import frc.robot.commands.ClimberDefault;
import frc.robot.commands.DashboardSpeedLauncherShoot;
import frc.robot.commands.DriveOverDistance;
import frc.robot.commands.DriveOverTime;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.DriverRelativeDrive;
import frc.robot.commands.DriverRelativeDriveAimAndLaunch;
import frc.robot.commands.DriverRelativeDriveWithAim;
import frc.robot.commands.FieldRelativeDrive;
import frc.robot.commands.HandBrake;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDefault;
import frc.robot.commands.LauncherDefault;
import frc.robot.commands.CameraLauncherShoot;
import frc.robot.commands.Output;
import frc.robot.commands.FeederDefault;
import frc.robot.commands.RobotRelativeDrive;
import frc.robot.commands.SetSpeedLauncherShoot;
import frc.robot.commands.SetFeederMode;
import frc.robot.commands.SwerveModuleTest;
import frc.robot.commands.SwerveMotorTest;
import frc.robot.commands.ZeroNavX;
import frc.robot.commands.auto.AutoDriverRelativeDrive;
import frc.robot.commands.auto.AutoRobotRelativeDrive;
import frc.robot.commands.auto.StartingPosition;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.FlywheelTable;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Feeder.FeederMode;
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
  private Climber m_climber = new Climber();
  private Camera m_camera = new Camera();
  private Launcher m_launcher = new Launcher();
  private Feeder m_feeder = new Feeder();
  private FlywheelTable m_flywheelTable = new FlywheelTable();
  private Intake m_intake = new Intake();

  private AutoBuilder m_autoBuilder = new AutoBuilder();
  public PowerDistribution m_pdp = new PowerDistribution();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_autoBuilder.registerCommand("robotRelativeDrive",
        (ParsedCommand pc) -> new AutoRobotRelativeDrive(pc, m_swerveDrive));
    m_autoBuilder.registerCommand("driverRelativeDrive",
        (ParsedCommand pc) -> new AutoDriverRelativeDrive(pc, m_swerveDrive));
    m_autoBuilder.registerCommand("launch",
        (ParsedCommand pc) -> new CameraLauncherShoot(m_launcher, m_camera, m_feeder, m_flywheelTable));
    m_autoBuilder.registerCommand("launchNoCamera",
        (ParsedCommand pc) -> SetSpeedLauncherShoot.CreateAutoCommand(pc, m_launcher, m_feeder));
    m_autoBuilder.registerCommand("startingPosition", (ParsedCommand pc) -> new StartingPosition(pc, m_swerveDrive));
    m_autoBuilder.registerCommand("driveToPosition",
        (ParsedCommand pc) -> DriveToPosition.CreateAutoCommand(pc, m_swerveDrive));
    m_autoBuilder.registerCommand("aimToGoal", (ParsedCommand pc) -> new AimToGoal(m_swerveDrive, m_camera));
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
    Command driverRelativeDrive = new DriverRelativeDrive(m_swerveDrive, m_driver);

    // Default Commands
    m_swerveDrive.setDefaultCommand(driverRelativeDrive);
    m_climber.setDefaultCommand(new ClimberDefault(m_climber, m_operator));
    m_intake.setDefaultCommand(new IntakeDefault(m_intake));
    m_launcher.setDefaultCommand(new LauncherDefault(m_launcher));
    m_feeder.setDefaultCommand(new FeederDefault(m_feeder));
    m_camera.setDefaultCommand(new CameraDefault(m_camera));

    configureMatchCommands();
    // configureDebugCommands();

  }

  private void configureDebugCommands() {
    m_driver.getButtonB().whileHeld(new AimToGoal(m_swerveDrive, m_camera));
    m_driver.getButtonRightStick().whileHeld(new DriverRelativeDriveWithAim(m_swerveDrive, m_driver, m_camera));
  }

  private void configureMatchCommands() {

    Command driverRelativeDrive = new DriverRelativeDrive(m_swerveDrive, m_driver);
    Command robotRelativeDrive = new RobotRelativeDrive(m_swerveDrive, m_driver);

    // Drive Commands
    m_driver.getButtonSelect().whenPressed(robotRelativeDrive);
    m_driver.getButtonStart().whenPressed(driverRelativeDrive);

    m_driver.getButtonLeftStick().whileHeld(new EnableSlowDriverSpeed(true));
    m_driver.getButtonRightStick().whileHeld(new DriverRelativeDriveWithAim(m_swerveDrive, m_driver, m_camera));

    // m_driver.getButtonA().whenPressed(new EnableSlowDriverSpeed(true));
    // m_driver.getButtonB().whenPressed(new EnableSlowDriverSpeed(false));
    m_driver.getButtonY().whileHeld(new DashboardSpeedLauncherShoot(m_launcher, m_feeder));

    m_driver.getButtonLB().whileHeld(() -> m_intake.MoveIntakeUp(), m_intake);
    m_driver.getButtonLT().whileHeld(new IntakeCommand(m_feeder, m_intake));

    m_driver.getButtonRB()
        .whileHeld(new SetSpeedLauncherShoot(m_launcher, m_feeder, Constants.DefaultLauncherHighSpeed, FeederMode.LAUNCH_HIGH_BUMPER));
    m_driver.getButtonRT()
        .whileHeld(new DriverRelativeDriveAimAndLaunch(m_swerveDrive, m_driver, m_camera, m_launcher, m_flywheelTable, m_feeder));

    m_driver.getPOVNorth().whileActiveOnce(new ZeroNavX(0, m_swerveDrive));
    m_driver.getPOVEast().whileActiveOnce(new ZeroNavX(90, m_swerveDrive));
    m_driver.getPOVSouth().whileActiveOnce(new ZeroNavX(180, m_swerveDrive));
    m_driver.getPOVWest().whileActiveOnce(new ZeroNavX(270, m_swerveDrive));
    m_driver.getButtonX().whileActiveContinuous(new HandBrake(m_swerveDrive));

    // Operator Commands
    m_operator.getButtonA().whileHeld(new IntakeCommand(m_feeder, m_intake));
    m_operator.getButtonB().whileHeld(new RunCommand(() -> m_intake.ManualIntake(1.0), m_intake)
        .alongWith(new SetFeederMode(m_feeder, FeederMode.BOTTOM_ONLY)));
    m_operator.getButtonX().whileHeld(new SetFeederMode(m_feeder, FeederMode.LAUNCH_CAMERA));
    m_operator.getButtonY().whileHeld(new Output(m_feeder, m_intake));

    m_operator.getButtonRB().whileHeld(new CameraLauncherShoot(m_launcher, m_camera, m_feeder, m_flywheelTable));
    m_operator.getButtonRT()
        .whileHeld(new SetSpeedLauncherShoot(m_launcher, m_feeder, Constants.DefaultLauncherLowSpeed, FeederMode.LAUNCH_LOW_BUMPER));

    m_operator.getButtonLB().whileHeld(new RunCommand(() -> m_intake.MoveIntakeUp(), m_intake));
    m_operator.getButtonLT().whileHeld(new RunCommand(() -> m_intake.MoveIntakeDown(), m_intake));

    m_operator.getPOVWest().whileHeld(new RunCommand(() -> m_climber.MoveArmUp(), m_climber));
    m_operator.getPOVEast().whileHeld(new RunCommand(() -> m_climber.MoveArmDown(), m_climber));
    m_operator.getPOVNorth().whileHeld(new RunCommand(() -> m_climber.ExtendToTop(), m_climber));
    m_operator.getPOVSouth().whileHeld(new RunCommand(() -> m_climber.ExtendToBottom(), m_climber));
  
    m_operator.getButtonStart().whileHeld(new RunCommand(() -> m_launcher.MoveHoodUp(), m_launcher));
    m_operator.getButtonSelect().whileHeld(new RunCommand(() -> m_launcher.MoveHoodDown(), m_launcher));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoBuilder.createAutoCommand();
  }
}
