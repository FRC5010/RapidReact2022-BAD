// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.mechanisms.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManyBallAuto extends SequentialCommandGroup {
  /** Creates a new ManyBallAuto. */
  public ManyBallAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    String path1 = "paths/LowestCargo.wpilib.json";
    String path2 = "paths/Ball1ToBall2.wpilib.json";
    addCommands(
            // new AutoIntakeDown(intake),
                new SequentialCommandGroup(
                    Drive.getAutonomousCommand(path1, true),
                    Drive.getAutonomousCommand(path2, false)));
  }
}