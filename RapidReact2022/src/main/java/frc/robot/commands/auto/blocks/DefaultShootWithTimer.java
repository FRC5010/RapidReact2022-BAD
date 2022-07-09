// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.blocks;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.DefaultShoot;
import frc.robot.commands.SpinTurret;
import frc.robot.commands.Timer;
import frc.robot.commands.VariableShot;
import frc.robot.constants.ShooterConstants;
import frc.robot.mechanisms.Transport;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DefaultShootWithTimer extends ParallelDeadlineGroup {
  /** Creates a new ShootWithDefaultShoot. */
  public DefaultShootWithTimer(Transport transport) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new Timer(3000));
    addCommands(
      new SpinTurret(transport.getTurretSubsystem(), transport.getShooterVision(), false),
      new VariableShot(transport.getShooterSubsystem(), transport.getVerticalIndexerSubsystem(), transport.getDiagonalIndexerSubsystem(),ShooterConstants.defenseRPM,ShooterConstants.defenseHood)
      );
    // addCommands(new FooCommand(), new BarCommand());
  }
}
