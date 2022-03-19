// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.pathing;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.mechanisms.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LowerBall2ToTerminal extends SequentialCommandGroup {
  /** Creates a new LowerBall2ToTerminal. */
  public LowerBall2ToTerminal() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    String path1 = "paths/LowerBall2ToTerminal.wpilib.json";
    addCommands(
      Drive.getAutonomousCommand(path1, true)
    );
  }
}
