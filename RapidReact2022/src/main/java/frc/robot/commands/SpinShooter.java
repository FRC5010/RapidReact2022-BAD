// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinShooter extends CommandBase { //Test for shooting
  /** Creates a new SpinShooter. */
  private ShooterSubsystem shooterSubsystem;
  private double flyWheelSpeed;
  private double feederSpeed;
  public SpinShooter(ShooterSubsystem shooterSubsystem, double flyWheelSpeed, double feederSpeed) {
    this.shooterSubsystem = shooterSubsystem;
    this.flyWheelSpeed = flyWheelSpeed;
    this.feederSpeed = feederSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.spinFlyWheel(flyWheelSpeed);
    shooterSubsystem.spinFeeder(feederSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
