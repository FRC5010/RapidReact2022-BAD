// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.HoodConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;

public class DefaultShoot extends CommandBase {
  /** Creates a new DefaultShoot. */
  private ShooterSubsystem shooterSubsystem;
  private VerticalIndexerSubsystem indexerSubsystem;
  public DefaultShoot(ShooterSubsystem shooterSubsystem, VerticalIndexerSubsystem indexerSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;


    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setFlyWheelPoint(ShooterConstants.defaultFlyWheelRPM);
    shooterSubsystem.spinFeeder(ShooterConstants.feederWheelPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      shooterSubsystem.spinUpWheelRPM();
      shooterSubsystem.determineIfReadyToShoot();
      if(shooterSubsystem.getReadyToShoot()){
        indexerSubsystem.setVerticalIndexer(ShooterConstants.indexerPow);
      }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.spinFeeder(0);
    shooterSubsystem.setFlyWheelPoint(0);
    shooterSubsystem.spinFlyWheel(0);
    indexerSubsystem.setVerticalIndexer(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
