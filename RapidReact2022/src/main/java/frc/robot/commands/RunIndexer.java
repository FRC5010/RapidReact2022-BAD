// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.UpperIndexerSubsystem;

public class RunIndexer extends CommandBase {
  /** Creates a new RunIndexer. */
  IndexerSubsystem indexerSubsystem;
  UpperIndexerSubsystem upperIndexerSubsystem; 
  Joystick operator;
  double power;

  public RunIndexer(UpperIndexerSubsystem upperIndexerSubsystem,IndexerSubsystem indexerSubsystem, double power) {
    this.indexerSubsystem = indexerSubsystem;
    this.upperIndexerSubsystem = upperIndexerSubsystem;
    this.power = power;    
    addRequirements(indexerSubsystem);
    addRequirements(upperIndexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    indexerSubsystem.setLowerIndexer(power);
    upperIndexerSubsystem.setUpperIndexer(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.setLowerIndexer(0);
    upperIndexerSubsystem.setUpperIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
