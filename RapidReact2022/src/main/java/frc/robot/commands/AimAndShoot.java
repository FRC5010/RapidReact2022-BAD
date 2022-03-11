// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.HoodConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UpperIndexerSubsystem;
import frc.robot.subsystems.vision.VisionSystem;

public class AimAndShoot extends CommandBase {
  /** Creates a new AimAndShoot. */
  private ShooterSubsystem shooterSubsystem;
  private UpperIndexerSubsystem indexerSubsystem;
  private VisionSystem visionSystem;

  public AimAndShoot(ShooterSubsystem shooterSubsystem, UpperIndexerSubsystem indexerSubsystem,
      VisionSystem visionSystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.visionSystem = visionSystem;

    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    shooterSubsystem.spinFeeder(ShooterConstants.feederWheelPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = visionSystem.getDistance() / 12.0;
    if (visionSystem.isValidTarget()) {
      double flyWheelPoint = shooterSubsystem.flyWheelCalculations(distance);
      shooterSubsystem.setFlyWheelPoint(flyWheelPoint);
      double hoodPoint = shooterSubsystem.hoodCalculations(distance);
      shooterSubsystem.setHoodSetPoint(hoodPoint);
      shooterSubsystem.spinUpWheelRPM();
      shooterSubsystem.pidHood();
    } 

    shooterSubsystem.determineIfReadyToShoot();

    if (shooterSubsystem.getReadyToShoot()) {
      indexerSubsystem.setUpperIndexer(ShooterConstants.indexerPow);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.spinFeeder(0);
    shooterSubsystem.setFlyWheelPoint(0);
    shooterSubsystem.spinFlyWheel(0);
    indexerSubsystem.setUpperIndexer(0);
    shooterSubsystem.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
