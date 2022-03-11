// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinIntake extends CommandBase {
  /** Creates a new SpinIntake. */
  private Joystick driver = null;
  private double power = 0.0;
  private IntakeSubsystem intakeSubsystem;
  private IndexerSubsystem indexerSubsystem;

  public SpinIntake(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, Joystick driver) {
    this.driver = driver;
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
      if (null != driver) { 
        intakePow = driver.getRawAxis(ControlConstants.intakeAxis) - driver.getRawAxis(ControlConstants.outtakeAxis);
        if(Math.abs(intakePow) != 0){
          intakeSubsystem.deployIntake();
        }else{
          intakeSubsystem.retractIntake();
        }
      } else {
        intakePow = power;
      }
    // modifies intake power cubing it and then using a multiplier
    double modPow = intakePow * 1.0;
    intakeSubsystem.setIntakePow(modPow);
    
    boolean opposingColor = intakeSubsystem.getColor().equals(ControlConstants.opposingColor);
    if(Math.abs(modPow) > 0 && !opposingColor){
      indexerSubsystem.setLowerIndexer(ShooterConstants.indexerPow);
    }else{
      indexerSubsystem.setLowerIndexer(0);
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
