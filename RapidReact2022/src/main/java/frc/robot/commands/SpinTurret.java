// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSystem;

public class SpinTurret extends CommandBase {
  /** Creates a new SpinTurret. */
  TurretSubsystem turretSubsystem;
  VisionSystem shooterSystem;
  Joystick operator;

  double lastAngle;
  double lastTime;
  public SpinTurret(TurretSubsystem turretSubsystem, VisionSystem shooterSystem,Joystick operator) {
    this.turretSubsystem = turretSubsystem;
    this.shooterSystem = shooterSystem;
    this.operator = operator;
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
    double pow = DriveTrainMain.scaleInputs(operator.getRawAxis(ControlConstants.operatorRightX));
    if(pow == 0){
      if(shooterSystem.isValidTarget()){
        double angle = shooterSystem.getAngleX();
        turretSubsystem.angleTurret(angle, lastTime, lastAngle);
        turretSubsystem.isOnTarget(angle);

        lastAngle = angle;
        lastTime = System.currentTimeMillis();
      }else{
        turretSubsystem.centerTurret();
        turretSubsystem.setOnTarget(false);
      }
    }else{
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
