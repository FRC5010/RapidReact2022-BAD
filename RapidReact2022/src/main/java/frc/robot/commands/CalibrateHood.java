// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class CalibrateHood extends CommandBase {
  /** Creates a new CalibrateHood. */
  ShooterSubsystem shooterSubsystem;
  private double currEncoderPos;
  private double lastEncoderPos;
  private int cycleCount = 0;

  public CalibrateHood(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setHoodMotor(-0.2);
    lastEncoderPos = 1000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setHoodCalibrated(true);
    shooterSubsystem.setHoodMotor(0);
    shooterSubsystem.setHoodEncoder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currEncoderPos = shooterSubsystem.getHoodPos();
    System.out.println("currEncoder" + currEncoderPos);
    if(Math.abs(shooterSubsystem.getHoodPos() - lastEncoderPos) < 0.1){
      if(cycleCount > 5){
        return true;
      }else{
        cycleCount++;
        return false;
      }
    }else{
      lastEncoderPos = currEncoderPos;
      cycleCount = 0;
      return false;
    }
  }
}
