/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.Controller;
import frc.robot.constants.ControlConstants;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.DriveTrainMain;

public class Driving extends CommandBase {
  DriveTrainMain driveSubsystem;
  Joystick driver;
  Controller driver2;
  /**
   * Creates a new Driving.
   */
  public Driving(DriveTrainMain driveTrain, Joystick driver, Controller driver2) {
    driveSubsystem = driveTrain;
    this.driver = driver;
    this.driver2 = driver2; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Drive.setCurrentLimits(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    driveSubsystem.driverArcadeDrive(
      // DriveTrainMain.scaleInputs(-driver.getRawAxis(ControlConstants.throttle)), 
      // DriveTrainMain.scaleInputs(driver.getRawAxis(ControlConstants.steer)));
      DriveTrainMain.scaleInputs(-driver2.getLeftYAxis()), 
      DriveTrainMain.scaleInputs(driver2.getRightXAxis()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drive.setCurrentLimits(ControlConstants.driveTrainCurrentLimit);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
