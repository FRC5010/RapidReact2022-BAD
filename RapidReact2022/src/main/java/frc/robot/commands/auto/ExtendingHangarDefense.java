// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.blocks.DefaultShootWithTimer;
import frc.robot.commands.auto.blocks.IntakeWithTimerBlock;
import frc.robot.commands.auto.blocks.MoveAndIntakeBlock;
import frc.robot.mechanisms.Transport;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendingHangarDefense extends SequentialCommandGroup {
  /** Creates a new ExtendingHangarDefense. */
  public ExtendingHangarDefense(Transport transport, SequentialCommandGroup drivingGroup) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveAndIntakeBlock(transport, drivingGroup, false),
      new IntakeWithTimerBlock(transport, 150), 
      new DefaultShootWithTimer(transport)
    );
  }
}
