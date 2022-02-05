// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.vision.VisionSystem;

public class SeekTarget extends CommandBase {
  /** Creates a new SeekTarget. */
  DriveTrainMain driveTrainMain;
  VisionSystem visionSystem;
  int seekDirection = 1;
  Pose pose;
  double angleToSwitchDir = 45;
  double startGyro;
  double currGyro;
  boolean targetFound = false;

  public SeekTarget(DriveTrainMain driveTrainMain, VisionSystem visionSystem, Pose pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainMain = driveTrainMain;
    this.visionSystem = visionSystem;
    this.pose = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleToSwitchDir = 45;
    targetFound = false;
    startGyro = pose.getHeading();
    angleToSwitchDir += startGyro;
    seekDirection = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currGyro = pose.getHeading();
    if(currGyro > angleToSwitchDir){
      seekDirection = -1;
    }
    driveTrainMain.arcadeDrive(0, seekDirection * .25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainMain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    targetFound = visionSystem.isValidTarget();
    return targetFound;
  }
}
