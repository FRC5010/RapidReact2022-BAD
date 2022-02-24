





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
import frc.robot.commands.FenderShot;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinTurret;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.UpperIndexerSubsystem;
import frc.robot.subsystems.vision.VisionSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HubToBall2 extends SequentialCommandGroup {
  /** Creates a new HubToBall2. */
  IntakeSubsystem intakeSubsystem;
  IndexerSubsystem indexerSubsystem;
  ShooterSubsystem shooterSubsystem;
  TurretSubsystem turretSubsystem;
  VisionSystem shooterVision;
  UpperIndexerSubsystem upperIndexerSubsystem;
  public HubToBall2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    String path1 = "paths/HubToBall2.wpilib.json";
    String path2 = "paths/Ball2ToHub.wpilib.json";

   addCommands(
      Drive.getAutonomousCommand(path1, true)
     );
  }
}
