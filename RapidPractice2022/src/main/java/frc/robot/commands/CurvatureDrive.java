package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.DriveTrainMain;

public class CurvatureDrive extends CommandBase {
  DriveTrainMain driveSubsystem;
  Joystick driver;
  /**
   * Creates a new Driving.
   */
  public CurvatureDrive(DriveTrainMain driveTrain, Joystick driver) {
    driveSubsystem = driveTrain;
    this.driver = driver;
    // Use addRequirements() here to declare subsystem dependenOcies.
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
    double throttle = -driver.getRawAxis(ControlConstants.throttle);
    double steer = driver.getRawAxis(ControlConstants.steer);
    SmartDashboard.putNumber("Throttle", throttle);
    SmartDashboard.putNumber("Steer", steer);
    //turns the robot at a curve/ rate like a car
    driveSubsystem.driverCurvatureDrive(
      DriveTrainMain.scaleInputs(throttle), 
      DriveTrainMain.scaleInputs(steer));
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