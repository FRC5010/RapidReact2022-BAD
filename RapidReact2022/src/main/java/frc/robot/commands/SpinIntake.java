// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinIntake extends CommandBase {
  /** Creates a new SpinIntake. */
  private Joystick operator;
  private IntakeSubsystem intakeSubsystem;

  public SpinIntake(IntakeSubsystem intakeSubsystem, Joystick operator) {
    this.operator = operator;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // gets intake pow from each analog trigger
    double intakePow = operator.getRawAxis(ControlConstants.intakeAxis) - operator.getRawAxis(ControlConstants.outtakeAxis);
    
    // modifies intake power cubing it and then using a multiplier
    double modPow = Math.pow(intakePow, 3) * 0.8;

    intakeSubsystem.setIntakePow(modPow);
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
