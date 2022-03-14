// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSystem;

public class DefaultClimb extends CommandBase {
  /** Creates a new DefaultClimb. */
  private ClimbSubsystem climbSubsystem;
  private Joystick operator;
  private IntakeSubsystem intakeSubsystem;
  private VisionSystem shooterVision;
  private ShooterSubsystem shooterSubsystem;
  private TurretSubsystem turretSubsystem;

  public DefaultClimb(ClimbSubsystem climbSubsystem, Joystick driver, IntakeSubsystem intakeSubsystem, VisionSystem shooterVision, ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem) {
    this.climbSubsystem = climbSubsystem;
    this.operator = driver;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterVision = shooterVision;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    addRequirements(climbSubsystem, intakeSubsystem, shooterSubsystem, turretSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterVision.setLight(false);
  }

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

    
    intakeSubsystem.deployIntake();

    // right climb
    climbSubsystem.setRightWinchSpeed(DriveTrainMain.scaleInputs(-operator.getRawAxis(ControlConstants.rightClimbArm)));

    // left climb
    climbSubsystem.setLeftWinchSpeed(DriveTrainMain.scaleInputs(-operator.getRawAxis(ControlConstants.leftClimbArm)));


    /*
    if(operator.getRawButton(ControlConstants.rightClimbArmUp) && 
    climbSubsystem.getRightEncoderValue() < ClimbConstants.climbBothMax){
        climbSubsystem.setRightWinchSpeed(ClimbConstants.climbSpeedUp);
    }else if(operator.getRawButton(ControlConstants.rightClimbArmDown)){
      climbSubsystem.setRightWinchSpeed(ClimbConstants.climbSpeedDown);
    }else{
      climbSubsystem.setRightWinchSpeed(0);
    }

    // left arm climb
    if(operator.getRawButton(ControlConstants.leftClimbArmUp) &&
    climbSubsystem.getLeftEncoderValue() < ClimbConstants.climbBothMax){
      climbSubsystem.setLeftWinchSpeed(ClimbConstants.climbSpeedUp);
    }else if(operator.getRawButton(ControlConstants.leftClimbArmDown)){
      climbSubsystem.setLeftWinchSpeed(ClimbConstants.climbSpeedDown);
    }else{
      climbSubsystem.setLeftWinchSpeed(0);
    }

    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.setLeftWinchSpeed(0);
    climbSubsystem.setRightWinchSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
