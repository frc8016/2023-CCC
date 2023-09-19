// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.ShooterConstants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DriveTrain m_DriveTrain = new DriveTrain();

  private final Joystick m_Joystick = new Joystick(0);

  private final Shooter m_Shooter = new Shooter();

  private final CommandXboxController m_XboxController = new CommandXboxController(1);  





  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_DriveTrain.setDefaultCommand(new RunCommand(() -> m_DriveTrain.arcadeDrive(m_Joystick), m_DriveTrain));}
    

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
    
    m_XboxController.rightBumper().whileTrue(new StartEndCommand(() -> m_Shooter.ShootCube(ShooterConstants.outerSpeed, ShooterConstants.innerSpeed), () -> m_Shooter.ShootCube(0,0), m_Shooter));
    m_XboxController.leftBumper().whileTrue(new StartEndCommand(() -> m_Shooter.IntakeCube(ShooterConstants.outerSpeedReversed, ShooterConstants.innerSpeedReversed), () -> m_Shooter.IntakeCube(0, 0), m_Shooter));
    //m_XboxController.rightBumper().whileTrue(new StartEndCommand(() -> m_Shooter.runShooter(ShooterConstants.innerSpeed), () -> m_Shooter.runShooter(0), m_Shooter));
    //m_XboxController.leftBumper().whileTrue(new StartEndCommand(() -> m_Shooter.runShooter(ShooterConstants.frontShooterSpeedReverse), () -> m_Shooter.runShooter(0), m_Shooter));

    //m_XboxController.leftTrigger().whileTrue(new StartEndCommand(() -> m_Shooter.runIndex(ShooterConstants.outerSpeed), () -> m_Shooter.runIndex(0), m_Shooter));
   // m_XboxController.rightTrigger().whileTrue(new StartEndCommand(() -> m_Shooter.runIndex(ShooterConstants.backIndexSpeedReverse), () -> m_Shooter.runIndex(0), m_Shooter));


    m_XboxController.leftTrigger().whileTrue(new StartEndCommand(() -> m_Shooter.IntakeCone(ShooterConstants.innerSpeed, ShooterConstants.outerSpeedReversed), () -> m_Shooter.IntakeCone(0, 0), m_Shooter));
    m_XboxController.rightTrigger().whileTrue(new StartEndCommand(() -> m_Shooter.ShootCone(ShooterConstants.innerSpeedReversed, ShooterConstants.outerSpeed), () -> m_Shooter.ShootCone(0, 0), m_Shooter));



    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
