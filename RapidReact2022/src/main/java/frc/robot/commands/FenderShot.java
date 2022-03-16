// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.FeederConstants;
import frc.robot.constants.ShooterConstants.HoodConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;

public class FenderShot extends CommandBase {
  /** Creates a new FenderShot. */
  private ShooterSubsystem shooterSubsystem;
  private VerticalIndexerSubsystem indexerSubsystem;
  private Joystick driver;
  private boolean isUpperShot = true;
  public FenderShot(ShooterSubsystem shooterSubsystem, VerticalIndexerSubsystem indexerSubsystem, Joystick driver) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.driver = driver;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public FenderShot(ShooterSubsystem shooterSubsystem, VerticalIndexerSubsystem indexerSubsystem, boolean isUpperShot) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.isUpperShot = isUpperShot;
    this.driver = null;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(driver != null)
      isUpperShot = driver.getRawButton(ControlConstants.upperFender);

    if (isUpperShot){
      shooterSubsystem.setFlyWheelPoint(ShooterConstants.highRPM);
      shooterSubsystem.setHoodSetPoint(HoodConstants.highHood);
    
    } else {
      shooterSubsystem.setFlyWheelPoint(ShooterConstants.lowRPM);
      shooterSubsystem.setHoodSetPoint(HoodConstants.lowHood);
    }
    
    shooterSubsystem.setFlyFeederPoint(FeederConstants.feederWheelRPM);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.spinUpWheelRPM();
    shooterSubsystem.pidHood();
    shooterSubsystem.determineIfReadyToShoot();
    shooterSubsystem.spinUpFeederRPM();

    if(shooterSubsystem.getReadyToShoot()){
      indexerSubsystem.setVerticalIndexerPoint(IndexerConstants.indexerRPM);
      indexerSubsystem.runWithVelocityControl();

    }else {
      indexerSubsystem.setVerticalIndexerPoint(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.spinFeeder(0);
    shooterSubsystem.setFlyWheelPoint(0);
    shooterSubsystem.spinFlyWheel(0);
    shooterSubsystem.stopHood();
    indexerSubsystem.setVerticalIndexer(0);
    
    shooterSubsystem.setFlyFeederPoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
