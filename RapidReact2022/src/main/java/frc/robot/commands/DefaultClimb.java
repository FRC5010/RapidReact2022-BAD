// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultClimb extends CommandBase {
  /** Creates a new DefaultClimb. */
  private ClimbSubsystem climbSubsystem;
  private Joystick driver;
  private Joystick operator;
  public DefaultClimb(ClimbSubsystem climbSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem,Joystick operator) {
    this.climbSubsystem = climbSubsystem;
    this.operator = operator;
    addRequirements(climbSubsystem, intakeSubsystem, shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.setStaticHookSpeed(operator.getRawAxis(ControlConstants.intakeAxis)-operator.getRawAxis(ControlConstants.outtakeAxis));
    climbSubsystem.rightWinch((-operator.getRawAxis(ControlConstants.spinHood)));
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
