// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.DriveTrainMain;

public class DriveTrainYEET extends CommandBase {
  /** Creates a new DriveTrainYEET. */
  double currTrottle;
  public DriveTrainYEET() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currTrottle = DriveConstants.throttleFactor;
    DriveConstants.throttleFactor = 1;
    Drive.setCurrentLimits(80);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveConstants.throttleFactor = currTrottle;
    Drive.setCurrentLimits(ControlConstants.driveTrainCurrentLimit);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
