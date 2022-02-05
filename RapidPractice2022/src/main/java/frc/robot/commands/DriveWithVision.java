/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveTrainMain;

import frc.robot.subsystems.vision.VisionSystem;

public class DriveWithVision extends CommandBase {
  
  // Drive with vision TURNS the robot till it sees it and DRIVES the robot to set distance

  DriveTrainMain drive;
  VisionSystem vision;
  

  double targetAngle, targetDistance, driveSpeed;
  double angleError, distanceError;
  double angleTolerance = 2;
  double distanceTolerance = 5;
  double startTime, currTime;
  boolean timeStarted;

  public DriveWithVision(DriveTrainMain drive, VisionSystem vision, double targetAngle, double targetDistance, double driveSpeed) {
    this.drive = drive;
    this.vision = vision;
    this.targetDistance = targetDistance;
    this.driveSpeed = driveSpeed;
    addRequirements(drive);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleError = vision.getRawValues().getAngleX() - targetAngle;
    distanceError = vision.getRawValues().getDistance() - targetDistance;
    // SmartDashboard.putNumber("intakeVisionAngleError", angleError);
    // SmartDashboard.putNumber("intakeVisionDistanceError", distanceError);

    double turnCorrection = angleError * DriveConstants.kTurnP * -1;
    double driveCorrection = distanceError * 0.0254 * DriveConstants.kPDriveVel / 12;
    //drive.arcadeDrive(driveCorrection + Math.signum(driveCorrection) * driveSpeed,
    System.out.println("turnpow: " + turnCorrection);
    drive.arcadeDrive(driveSpeed, 
      turnCorrection + Math.signum(turnCorrection) *  DriveConstants.minTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!vision.isValidTarget()){
      if(!timeStarted){
        startTime = System.currentTimeMillis();
        timeStarted = true;
      }
      currTime = System.currentTimeMillis();
      if(currTime - startTime >= 2000){
        return true;
      }
    }
    return false;
  }
}