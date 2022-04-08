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
  public LockAndLoad(VerticalIndexerSubsystem verticalSubsystem, DiagonalIndexerSubsystem diagonalSubsystem, ShooterSubsystem shooterSubsystem, VisionSystem shooterVision) {
    // Use addRequirements() here to declare subsystem dependencies.
      this.verticalSubsystem = verticalSubsystem;
      this.diagonalSubsystem = diagonalSubsystem;
      this.shooterSubsystem = shooterSubsystem;
      this.shooterVision = shooterVision;

      addRequirements(verticalSubsystem, shooterSubsystem, diagonalSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    diagonalBB = diagonalSubsystem.getLowerBB();
    shooterSubsystem.setFlyFeederPoint(ShooterConstants.defaultFlyWheelRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = shooterVision.getDistance() / 12.0;
    if (shooterVision.isValidTarget()) {
      double flyWheelPoint = shooterSubsystem.flyWheelCalculations(distance);
      shooterSubsystem.setFlyWheelPoint(flyWheelPoint);

      shooterSubsystem.runFlyWheelWithVelocityControl();
      double hoodPoint = shooterSubsystem.hoodCalculations(distance);
      shooterSubsystem.setHoodSetPoint(hoodPoint);
      shooterSubsystem.pidHood();
      shooterSubsystem.runFeederWheelWithVelocityControl();     

    } 
    diagonalBB = diagonalSubsystem.getLowerBB();
    if (!diagonalBB) {
      System.out.println("down ooo");
      diagonalSubsystem.setDiagonalIndexer(-0.35);
      verticalSubsystem.setVerticalIndexer(-0.5);  
    } else {
      System.out.println("no down");
      diagonalSubsystem.setDiagonalIndexer(0);
      verticalSubsystem.setVerticalIndexer(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    verticalSubsystem.setVerticalIndexer(0);
    diagonalSubsystem.setDiagonalIndexer(0);
    System.out.println("L&L Ended " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
