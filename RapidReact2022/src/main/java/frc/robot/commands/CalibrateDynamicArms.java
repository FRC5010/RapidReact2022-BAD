// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.mechanisms.Climb;
import frc.robot.subsystems.ClimbSubsystem;

public class CalibrateDynamicArms extends CommandBase {
  /** Creates a new CalibrateArms. */
  ClimbSubsystem climbSubsystem; 
  private double startRight;
  private double startLeft;
  private boolean doneRight = false; 
  private boolean doneLeft = false;

  public CalibrateDynamicArms(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startLeft = climbSubsystem.getLeftEncoderValue();
    startRight = climbSubsystem.getRightEncoderValue();
    climbSubsystem.setLeftWinchSpeed(ClimbConstants.calibrateDown);
    climbSubsystem.setRightWinchSpeed(ClimbConstants.calibrateDown);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currLeft = climbSubsystem.getLeftEncoderValue();
    double currRight = climbSubsystem.getRightEncoderValue();
    if(Math.abs(currLeft - startLeft) > ClimbConstants.climbBothMax - 5){
      climbSubsystem.setLeftWinchSpeed(0);
      doneLeft = true;
    }
    if(Math.abs(currRight - startRight) > ClimbConstants.climbBothMax - 5){
      climbSubsystem.setRightWinchSpeed(0);
      doneRight = true;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.setRightWinchSpeed(0);
    climbSubsystem.setLeftWinchSpeed(0);
    climbSubsystem.zeroArms();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return doneLeft && doneRight;
  }
}
