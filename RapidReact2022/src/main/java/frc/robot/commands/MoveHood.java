// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveHood extends CommandBase {
  /** Creates a new MoveHood. */
  private ShooterSubsystem shooterSubsystem;
  double moveNum;

  public MoveHood(ShooterSubsystem shooterSubsystem, double moveNum) {
    this.shooterSubsystem = shooterSubsystem;
    this.moveNum = moveNum;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currPos = shooterSubsystem.getHoodSetPoint();
    shooterSubsystem.setHoodSetPoint(currPos + moveNum);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.pidHood();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(shooterSubsystem.getHoodPos() - shooterSubsystem.getHoodSetPoint()) < ShooterConstants.HoodConstants.shootingHoodTolerance;
  }
}
