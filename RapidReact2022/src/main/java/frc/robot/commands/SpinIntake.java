// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.DiagonalIndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;

public class SpinIntake extends CommandBase {
  /** Creates a new SpinIntake. */
  private Joystick driver = null;
  private double power = 0.0;
  private IntakeSubsystem intakeSubsystem;
  private DiagonalIndexerSubsystem indexerSubsystem;
  private VerticalIndexerSubsystem upperIndexerSubsystem;

  //creating delay for outtaking wrong balls
  private boolean rejectBall = false;
  private long delayMS;
  private long startTime;
  private long currTime;

  public SpinIntake(IntakeSubsystem intakeSubsystem, DiagonalIndexerSubsystem indexerSubsystem, VerticalIndexerSubsystem upperIndexerSubsystem, Joystick driver, long delayMS) {
    this.driver = driver;
    this.intakeSubsystem = intakeSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.upperIndexerSubsystem = upperIndexerSubsystem;
    this.delayMS = delayMS;
    addRequirements(intakeSubsystem, indexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SpinIntake(IntakeSubsystem intakeSubsystem, DiagonalIndexerSubsystem indexerSubsystem, Double power) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.power = power;
    addRequirements(intakeSubsystem, indexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.deployIntake();
    startTime = System.currentTimeMillis();
    rejectBall = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // gets intake pow from each analog trigger
    double intakePow;
      if (null != driver) { 
        intakePow = driver.getRawAxis(ControlConstants.intakeAxis) - driver.getRawAxis(ControlConstants.outtakeAxis);
      } else {
        intakePow = power;
      }
    // modifies intake power cubing it and then using a multiplier
    double modPow = intakePow * 1.0;
    intakeSubsystem.setIntakePow(modPow);
    
    currTime = System.currentTimeMillis();
    boolean opposingColor = intakeSubsystem.getColor().equals(ControlConstants.opposingColor);
      double confidence = intakeSubsystem.getConfidence();
    //should reject ball for a minimum of delayMS(for now that is 200 ms) before allowing the intake to spin properly
    if(rejectBall == true && intakeSubsystem.getRejectState() == true){
      if(currTime - startTime >= delayMS){
        rejectBall = false;
        startTime = currTime;
      }else{
        indexerSubsystem.setDiagonalIndexerPoint(-IndexerConstants.indexerRPM);
      }
    }else{
      if(confidence > 0.90){
        if(!opposingColor){
          indexerSubsystem.setDiagonalIndexerPoint(Math.signum(modPow) * IndexerConstants.indexerRPM);
        }else{
          //indexerSubsystem.setDiagonalIndexerPoint(-IndexerConstants.indexerRPM);
          rejectBall = true;
          startTime = currTime;
        }
      }else{
        indexerSubsystem.setDiagonalIndexerPoint(Math.signum(modPow) * IndexerConstants.indexerRPM);
      }
    }
    indexerSubsystem.runWithVelocityControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.setDiagonalIndexerPoint(0);
    indexerSubsystem.setDiagonalIndexer(0);

    intakeSubsystem.setIntakePow(0);

    intakeSubsystem.retractIntake();

    //CommandScheduler.getInstance().schedule(new ParallelDeadlineGroup(new Timer(200), new RunIndexer(upperIndexerSubsystem, indexerSubsystem, IndexerConstants.indexerRPM)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
