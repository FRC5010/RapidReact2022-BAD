// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultClimb extends CommandBase {
  /** Creates a new DefaultClimb. */
  private ClimbSubsystem climbSubsystem;
  private Joystick driver;
  private IntakeSubsystem intakeSubsystem;

  public DefaultClimb(ClimbSubsystem climbSubsystem, Joystick driver, IntakeSubsystem intakeSubsystem) {
    this.climbSubsystem = climbSubsystem;
    this.driver = driver;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(climbSubsystem, intakeSubsystem);
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
    // static arms up and down
    //climbSubsystem.setStaticHookSpeed(driver.getRawAxis(ControlConstants.staticHookUp)-driver.getRawAxis(ControlConstants.staticHookDown));
    //static hooks are remaining up since they kept getting stuck
    //driver triggers are now set to outtake and intake

    // right arm climb
    intakeSubsystem.deployIntake();
    if(driver.getRawButton(ControlConstants.rightClimbArmUp) && 
    climbSubsystem.getRightEncoderValue() < ClimbConstants.climbBothMax){
        climbSubsystem.setRightWinchSpeed(ClimbConstants.climbSpeedUp);
    }else if(driver.getRawButton(ControlConstants.rightClimbArmDown)){
      climbSubsystem.setRightWinchSpeed(ClimbConstants.climbSpeedDown);
    }else{
      climbSubsystem.setRightWinchSpeed(0);
    }

    // left arm climb
    if(driver.getRawButton(ControlConstants.leftClimbArmUp) &&
    climbSubsystem.getLeftEncoderValue() < ClimbConstants.climbBothMax){
      climbSubsystem.setLeftWinchSpeed(ClimbConstants.climbSpeedUp);
    }else if(driver.getRawButton(ControlConstants.leftClimbArmDown)){
      climbSubsystem.setLeftWinchSpeed(ClimbConstants.climbSpeedDown);
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
