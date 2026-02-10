// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.commands.alignDistanceWithTagCommand;
import frc.robot.cwtech.Conditioning;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootingCommand;
import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Conditioning m_driveXConditioning = new Conditioning();
  private final Conditioning m_driveYConditioning = new Conditioning();
  private final Conditioning m_turnConditioning = new Conditioning();

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShootingSubsystem m_ShootingSubsystem = new ShootingSubsystem();
  private final SendableChooser<Command> autoChooser;

  public boolean fieldRelative = true;
  private final Robot m_robot;
  private TeleOpDriveCommand m_TeleOpDriveCommand;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort0);
      public final static CommandXboxController m_subDriverController = new CommandXboxController(
      OIConstants.kDriverControllerPort1);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    m_robot = robot;

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

     m_TeleOpDriveCommand=new TeleOpDriveCommand(m_DriveSubsystem,
      () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(),
       () -> m_robot.isTeleopEnabled(),()->fieldRelative);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.start().onTrue(new
      InstantCommand(()->m_DriveSubsystem.zeroHeading()));
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    m_DriveSubsystem.setDefaultCommand(m_TeleOpDriveCommand);
    m_driverController.a().onTrue(new alignDistanceWithTagCommand(m_DriveSubsystem
    ));

    m_driverController.y().onTrue(new InstantCommand(() -> fieldRelative = false));
     m_driverController.x().onTrue(new InstantCommand(() -> fieldRelative = true));
    m_driverController.leftBumper().onTrue(new InstantCommand(() ->m_TeleOpDriveCommand.flipBump()));
     m_driverController.rightBumper().whileTrue(new InstantCommand(() -> m_speedMultiplier = 0.5));
     m_driverController.rightBumper().whileFalse(new InstantCommand(() -> m_speedMultiplier = 1.0));
     m_driverController.back().onTrue(new InstantCommand(() -> m_DriveSubsystem.stopAndLockWheels()));
     m_subDriverController.a().whileTrue(new IntakeCommand(m_IntakeSubsystem));
     m_subDriverController.b().whileTrue(new ShootingCommand(m_ShootingSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
  return autoChooser.getSelected();  
}

  public void initalize()
  {
    m_DriveSubsystem.initalize();
  }

  private double m_speedMultiplier = 1.0;
  public double getDriveXInput()
  {
    // We getY() here because of the FRC coordinate system being turned 90 degrees
    return m_driveXConditioning.condition(-m_driverController.getLeftY())
            * DriveSubsystem.kMaxSpeedMetersPerSecond
            * m_speedMultiplier;
  }

  public double getDriveYInput()
  {
    // We getX() here becasuse of the FRC coordinate system being turned 90 degrees
    return m_driveYConditioning.condition(-m_driverController.getLeftX())
            * DriveSubsystem.kMaxSpeedMetersPerSecond
            * m_speedMultiplier;
  }

  public double getTurnInput()
  {
    return m_turnConditioning.condition(-m_driverController.getRightX())
            * DriveSubsystem.kMaxAngularSpeedRadiansPerSecond
            * m_speedMultiplier;
  }
}
