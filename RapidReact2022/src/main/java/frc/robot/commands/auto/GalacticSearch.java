/*
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimWithVision;
import frc.robot.commands.DriveWithVision;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.SeekTarget;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.ShaftSubsystem;
import frc.robot.subsystems.VisionSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearch extends SequentialCommandGroup {
  private DriveTrainMain driveTrainMain;
  private VisionSystem visionSystem;
  private Pose pose;
  private IntakeSubsystem intakeSubsystem;
  private ShaftSubsystem shaftSubsystem;


  public GalacticSearch(DriveTrainMain driveTrainMain, VisionSystem visionSystem, Pose pose, IntakeSubsystem intakeSubsystem, ShaftSubsystem shaftSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainMain = driveTrainMain;
    this.visionSystem = visionSystem;
    this.pose = pose;
    this.intakeSubsystem = intakeSubsystem;
    this.shaftSubsystem = shaftSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new SeekTarget(this.driveTrainMain, this.visionSystem, this.pose), 
    new ParallelDeadlineGroup(
      new DriveWithVision(this.driveTrainMain, this.visionSystem, this.shaftSubsystem, 0, 0, -.25),
      new IntakeBalls(this.intakeSubsystem, .8)
      ),
      new SeekTarget(this.driveTrainMain, this.visionSystem, this.pose), 
    new ParallelDeadlineGroup(
      new DriveWithVision(this.driveTrainMain, this.visionSystem, this.shaftSubsystem, 0, 0, -.25),
      new IntakeBalls(this.intakeSubsystem, .8)
      ),
      new SeekTarget(this.driveTrainMain, this.visionSystem, this.pose), 
    new ParallelDeadlineGroup(
      new DriveWithVision(this.driveTrainMain, this.visionSystem, this.shaftSubsystem, 0, 0, -.25),
      new IntakeBalls(this.intakeSubsystem, .8)
      )
    );
  }
}
*/
