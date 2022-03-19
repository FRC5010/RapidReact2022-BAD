// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.blocks;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.Timer;
import frc.robot.mechanisms.Transport;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeWithTimerBlock extends ParallelDeadlineGroup {
  /** Creates a new IntakeWithTimer. */
  public IntakeWithTimerBlock(Transport transport, long timeMilli) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new Timer(timeMilli));
    addCommands(new SpinIntake(transport.getIntakeSubsystem(), transport.getDiagonalIndexerSubsystem(), 1.0));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
