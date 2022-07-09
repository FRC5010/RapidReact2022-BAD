// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.CalibrateHood;
import frc.robot.commands.VariableShot;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinTurret;
import frc.robot.commands.Timer;
import frc.robot.commands.auto.blocks.CalibrateSystemBlock;
import frc.robot.commands.auto.blocks.MoveAndIntakeBlock;
import frc.robot.commands.auto.blocks.ShootWithTimerBlock;
import frc.robot.mechanisms.Transport;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TarmacTwoBall extends SequentialCommandGroup {
  /** Creates a new AutoShootOnly. */
  public TarmacTwoBall(Transport transport, SequentialCommandGroup drivingGroup) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CalibrateSystemBlock(transport),

      new MoveAndIntakeBlock(transport, drivingGroup, false),

      new ShootWithTimerBlock(transport, 1500, false)
    
    );

  }
}
