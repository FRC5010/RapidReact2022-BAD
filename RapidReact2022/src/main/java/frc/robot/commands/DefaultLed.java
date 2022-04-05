// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mechanisms.Transport;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSystem;

public class DefaultLed extends CommandBase {
  /** Creates a new DefaultLed. */
  LedSubsystem ledSubsystem;
  TurretSubsystem turretSubsystem;
  ShooterSubsystem shooterSubsystem;
  VisionSystem visionSystem;
  Transport transport; 
  public DefaultLed(LedSubsystem ledSubsystem, Transport transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledSubsystem = ledSubsystem;
    this.transport = transport;
    turretSubsystem = transport.getTurretSubsystem();
    shooterSubsystem = transport.getShooterSubsystem();
    visionSystem = transport.getShooterVision();

    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.setSolidColor(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledSubsystem.setSolidColor(0, 0, 0);  
    if (visionSystem.isValidTarget()){
      ledSubsystem.setSolidColor(0, 0, 255);
    }

    if (TurretSubsystem.getIsOnTarget()){
      ledSubsystem.setSolidColor(255, 25, 0);
    }

    if (shooterSubsystem.getReadyToShoot() && TurretSubsystem.getIsOnTarget()){
      ledSubsystem.setSolidColor(255, 0, 0);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
