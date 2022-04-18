// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.blocks;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.CalibrateHood;
import frc.robot.commands.Timer;
import frc.robot.mechanisms.Transport;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CalibrateSystemBlock extends ParallelCommandGroup {
  /** Creates a new CalibrateSystems. */
  public CalibrateSystemBlock(Transport transport) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new Timer(750), 
        new CalibrateHood(transport.getShooterSubsystem())
      ),
      new InstantCommand(() -> transport.getIntakeSubsystem().deployIntake()),
      new InstantCommand(() -> transport.getTurretSubsystem().zeroTurret())
    );
  }
}
