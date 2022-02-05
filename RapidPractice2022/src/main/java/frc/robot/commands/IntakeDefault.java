// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.PneumaticSubsystem;

public class IntakeDefault extends CommandBase {
  private PneumaticSubsystem intake;
  private Joystick joystick;

  /** Creates a new IntakeDefault. */
  public IntakeDefault(PneumaticSubsystem intake, Joystick joystick) 
  {
    this.intake = intake;
    this.joystick = joystick;
    addRequirements(intake);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //Joystick Power Curve    
    double Power = joystick.getRawAxis(ControlConstants.intakeAxis) - joystick.getRawAxis(ControlConstants.outtakeAxis);
    Power = Math.pow(Power, 3) * 0.8;
    intake.intake(Power);
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
