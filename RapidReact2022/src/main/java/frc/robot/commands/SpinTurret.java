// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.TurretConstants;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSystem;

public class SpinTurret extends CommandBase {
  /** Creates a new SpinTurret. */
  TurretSubsystem turretSubsystem;
  VisionSystem shooterSystem;
  Joystick operator = null;
  boolean isSeek = false;
  double lastAngle;
  double lastTime;
  int seekDirection = -1; 
  public SpinTurret(TurretSubsystem turretSubsystem, VisionSystem shooterSystem,Joystick operator, boolean isSeek) {
    this.turretSubsystem = turretSubsystem;
    this.shooterSystem = shooterSystem;
    this.operator = operator;
    this.isSeek = isSeek; 
    addRequirements(turretSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SpinTurret(TurretSubsystem turretSubsystem, VisionSystem shooterSystem, boolean isSeek) {
    this.turretSubsystem = turretSubsystem;
    this.shooterSystem = shooterSystem;
    this.isSeek = isSeek; 
    addRequirements(turretSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTime = System.currentTimeMillis();
    lastAngle = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pow = 0;
    if (null != operator) {
      pow = DriveTrainMain.scaleInputs(operator.getRawAxis(ControlConstants.turnTurret)); 
    } 
    if(pow == 0){
      if(shooterSystem.isValidTarget()){
        // pid turret
        double angle = shooterSystem.getAngleX();
        turretSubsystem.angleTurret(angle, lastTime, lastAngle);
        turretSubsystem.isOnTarget(angle);

        lastAngle = angle;
        lastTime = System.currentTimeMillis();
      }else{
        // seeking code
        if (isSeek){
          pow = TurretConstants.seekSpeed; 
          turretSubsystem.turnTurret(pow * seekDirection);
          if (turretSubsystem.isAtRightLimit()){
            seekDirection = -1;
          }
          if (turretSubsystem.isAtLeftLimit()){
            seekDirection = 1; 
          }
        } else {
          turretSubsystem.centerTurret();
          turretSubsystem.setOnTarget(false);
        }
      }
    }else{
      // this runs on the joystick
      turretSubsystem.turnTurret(pow);
      turretSubsystem.setOnTarget(false);
      
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
