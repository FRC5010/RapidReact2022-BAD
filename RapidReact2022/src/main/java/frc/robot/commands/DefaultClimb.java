// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class DefaultClimb extends CommandBase {
  /** Creates a new DefaultClimb. */
  private ClimbSubsystem climbSubsystem;
  private Joystick driver;

  public DefaultClimb(ClimbSubsystem climbSubsystem, Joystick driver) {
    this.climbSubsystem = climbSubsystem;
    this.driver = driver;
    addRequirements(climbSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  //y and x are left side climb
  //b and a are right side climb
  //y and b go up
  //x and a go down
  @Override
  public void execute() {
    climbSubsystem.setStaticHookSpeed(driver.getRawAxis(ControlConstants.staticHookUp)-driver.getRawAxis(ControlConstants.staticHookDown));
    
    if(driver.getRawButton(ControlConstants.rightClimbArmUp)){
      climbSubsystem.setRightWinchSpeed(1);
    }else if(driver.getRawButton(ControlConstants.rightClimbArmDown)){
      climbSubsystem.setRightWinchSpeed(-1);
    }else{
      climbSubsystem.setRightWinchSpeed(0);
    }
    if(driver.getRawButton(ControlConstants.leftClimbArmUp)){
      climbSubsystem.setLeftWinchSpeed(1);
    }else if(driver.getRawButton(ControlConstants.leftClimbArmDown)){
      climbSubsystem.setLeftWinchSpeed(-1);
    }else{
      climbSubsystem.setLeftWinchSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.setLeftWinchSpeed(0);
    climbSubsystem.setRightWinchSpeed(0);
    climbSubsystem.setStaticHookSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
