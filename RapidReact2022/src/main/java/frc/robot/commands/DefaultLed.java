// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.naming.ldap.LdapReferralException;

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
    //ledSubsystem.orbit(255, 20, 0, 0, 0, 255, .25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int color = 0;
    if (visionSystem.isValidTarget()){
      color = 1;
    }

    if (TurretSubsystem.getIsOnTarget()){
      color = 2; 
    }

    if (shooterSubsystem.getFlywheelSetPoint() > 0){
      color = 3; 
    }



    switch(color){
      case 1: 
        ledSubsystem.setSolidColor(0, 0, 255);
        break;
      case 2: 
        ledSubsystem.setSolidColor(255, 20, 0);
        break;
      case 3:
        double percent = shooterSubsystem.getFlyWheelPercentRPM();
        if(percent < 0.97){
          ledSubsystem.setSolidColorPercent(255, 20, 0, Math.pow(percent, 3));
        } else if (shooterSubsystem.getReadyToShoot()){
          ledSubsystem.setSolidColor(255, 0, 255);
        } else {
          ledSubsystem.setSolidColor(255, 0, 0);
        }
        break; 
      default: 
        ledSubsystem.setSolidColor(0, 0, 0);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //ledSubsystem.orbit(255, 20, 0, 0, 0, 255, .25);
    ledSubsystem.rainbow(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
