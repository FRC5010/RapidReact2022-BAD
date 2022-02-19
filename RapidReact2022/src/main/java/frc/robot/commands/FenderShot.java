// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.HoodConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UpperIndexerSubsystem;

public class FenderShot extends CommandBase {
  /** Creates a new FenderShot. */
  private ShooterSubsystem shooterSubsystem;
  private UpperIndexerSubsystem indexerSubsystem;
  private Joystick driver;
  public FenderShot(ShooterSubsystem shooterSubsystem, UpperIndexerSubsystem indexerSubsystem, Joystick driver) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.driver = driver;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (driver.getRawButton(ControlConstants.upperFender)){
      shooterSubsystem.setFlyWheelPoint(ShooterConstants.highRPM);
      shooterSubsystem.setHoodSetPoint(HoodConstants.highHood);
    
    } else {
      shooterSubsystem.setFlyWheelPoint(ShooterConstants.lowRPM);
      shooterSubsystem.setHoodSetPoint(HoodConstants.lowHood);
    }
    
    shooterSubsystem.spinFeeder(ShooterConstants.feederWheelPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.spinUpWheelRPM();
    shooterSubsystem.pidHood();
    shooterSubsystem.determineIfReadyToShoot();
    if(shooterSubsystem.getReadyToShoot()){
      indexerSubsystem.setUpperIndexer(ShooterConstants.indexerPow);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.spinFeeder(0);
    shooterSubsystem.setFlyWheelPoint(0);
    shooterSubsystem.spinFlyWheel(0);
    shooterSubsystem.stopHood();
    indexerSubsystem.setUpperIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
