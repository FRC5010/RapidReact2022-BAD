// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.DiagonalIndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;
import frc.robot.subsystems.vision.VisionSystem;

public class LockAndLoad extends CommandBase {
  /** Creates a new LockAndLoad. */
  VerticalIndexerSubsystem verticalSubsystem; 
  DiagonalIndexerSubsystem diagonalSubsystem; 
  ShooterSubsystem shooterSubsystem; 
  VisionSystem shooterVision;
  boolean diagonalBB; 
  boolean verticalBB; 
  public LockAndLoad(VerticalIndexerSubsystem verticalSubsystem, DiagonalIndexerSubsystem diagonalSubsystem, ShooterSubsystem shooterSubsystem, VisionSystem shooterVision) {
    // Use addRequirements() here to declare subsystem dependencies.
      this.verticalSubsystem = verticalSubsystem;
      this.diagonalSubsystem = diagonalSubsystem;
      this.shooterSubsystem = shooterSubsystem;
      this.shooterVision = shooterVision;

      addRequirements(verticalSubsystem, diagonalSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    diagonalBB = diagonalSubsystem.getLowerBB();
    verticalBB = verticalSubsystem.getUpperBB(); 
    shooterSubsystem.setFlyFeederPoint(ShooterConstants.defaultFlyWheelRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = shooterVision.getDistance() / 12.0;
    if (shooterVision.isValidTarget()) {
      double flyWheelPoint = shooterSubsystem.flyWheelCalculations(distance);
      shooterSubsystem.setFlyWheelPoint(flyWheelPoint);
      double hoodPoint = shooterSubsystem.hoodCalculations(distance);
      shooterSubsystem.setHoodSetPoint(hoodPoint);
      shooterSubsystem.spinUpWheelRPM();
      shooterSubsystem.pidHood();

      shooterSubsystem.spinUpFeederRPM();
    } 
    if (!verticalBB || !diagonalBB){
      verticalSubsystem.setVerticalIndexer(-0.5);
    }  else {
      verticalSubsystem.setVerticalIndexer(0);
    }

    if (!diagonalBB) {
      diagonalSubsystem.setDiagonalIndexer(-0.5);
    } else {
      diagonalSubsystem.setDiagonalIndexer(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    verticalSubsystem.setVerticalIndexer(0);
    diagonalSubsystem.setDiagonalIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
