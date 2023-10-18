// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static CommandBase scoreHigh(Shooter shooter, Arm arm) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              arm.setGoal(90);
              arm.enable();
            },
            arm),
        new WaitCommand(0),
        Commands.runOnce(
            (() ->
                shooter.ShootCone(
                    ShooterConstants.innerOuterSpeedReversed, ShooterConstants.innerOuterSpeed)),
            shooter),
        Commands.runOnce(
            () -> {
              arm.setGoal(null);
              arm.enable();
            }));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
