// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.DriveTrainMain;

public class ComboDrive extends CommandBase {
  /** Creates a new ComboDrive. */
  DriveTrainMain driveTrain;
  Joystick driver;
  public ComboDrive(DriveTrainMain driveTrain, Joystick driver) {
    this.driveTrain = driveTrain;
    this.driver = driver;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Drive.setCurrentLimits(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    double throttle = DriveTrainMain.scaleInputs(-driver.getRawAxis(ControlConstants.throttle));
    double steer = DriveTrainMain.scaleInputs(driver.getRawAxis(ControlConstants.steer));
    SmartDashboard.putNumber("Throttle", throttle);
    SmartDashboard.putNumber("Steer", steer);
    //driveSubsystem.curvatureDrive();

    //switches from arcade to curvature at a bound

    if(Math.abs(throttle) < .85){
      driveTrain.driverArcadeDrive(throttle, steer);
      SmartDashboard.putString("Mode", "arcade");
    }else{
      driveTrain.driverCurvatureDrive(throttle, steer, false);
      SmartDashboard.putString("Mode", "curve");
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
