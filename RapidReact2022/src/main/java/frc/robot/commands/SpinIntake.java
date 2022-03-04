// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinIntake extends CommandBase {
  /** Creates a new SpinIntake. */
  private Joystick operator = null;
  private double power = 0.0;
  private IntakeSubsystem intakeSubsystem;
  private IndexerSubsystem indexerSubsystem;

  public SpinIntake(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, Joystick operator) {
    this.operator = operator;
    this.intakeSubsystem = intakeSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(intakeSubsystem, indexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SpinIntake(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, Double power) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.power = power;
    addRequirements(intakeSubsystem, indexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // gets intake pow from each analog trigger
    double intakePow;
    if(intakeSubsystem.isIntakeDeployed()){
      if (null != operator) { 
        intakePow = operator.getRawAxis(ControlConstants.intakeAxis) - operator.getRawAxis(ControlConstants.outtakeAxis);
      } else {
        intakePow = power;
      }
    }else{
      intakePow = 0;
    }
    
    // modifies intake power cubing it and then using a multiplier
    double modPow = intakePow * 1.0;

    intakeSubsystem.setIntakePow(modPow);
    
    if(Math.abs(modPow) > 0){
      
      indexerSubsystem.setLowerIndexer(ShooterConstants.indexerPow);
    }else{
      //if(!indexerSubsystem.isLowerIndexerRunning()){
      indexerSubsystem.setLowerIndexer(0);
      //}
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
