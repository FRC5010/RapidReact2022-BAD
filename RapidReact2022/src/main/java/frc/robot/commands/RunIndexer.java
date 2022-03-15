// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DiagonalIndexerSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;

public class RunIndexer extends CommandBase {
  /** Creates a new RunIndexer. */
  DiagonalIndexerSubsystem indexerSubsystem;
  VerticalIndexerSubsystem upperIndexerSubsystem; 
  Joystick operator;
  double power;

  public RunIndexer(VerticalIndexerSubsystem upperIndexerSubsystem,DiagonalIndexerSubsystem indexerSubsystem, double power) {
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
    //indexerSubsystem.setDiagonalIndexer(power);
    //upperIndexerSubsystem.setVerticalIndexer(power);

    indexerSubsystem.setDiagonalIndexerPoint(power);
    upperIndexerSubsystem.setVerticalIndexerPoint(power);

    indexerSubsystem.spinUpDiagonalIndexerRPM();
    upperIndexerSubsystem.spinUpVerticalIndexerRPM();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.setDiagonalIndexerPoint(0);
    indexerSubsystem.setDiagonalIndexer(0);

    upperIndexerSubsystem.setVerticalIndexerPoint(0);
    upperIndexerSubsystem.setVerticalIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
