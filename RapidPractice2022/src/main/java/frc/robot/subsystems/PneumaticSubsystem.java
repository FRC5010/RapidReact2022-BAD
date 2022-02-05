// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeDefault;

public class PneumaticSubsystem extends SubsystemBase {
  DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  CANSparkMax intake = new CANSparkMax(8, MotorType.kBrushless);

  public PneumaticSubsystem() {

  }
  public void togglePiston(){
    piston.toggle();
  }
  public void pistonOff(){
    piston.set(Value.kOff);
  }
  public void pistonForward(){
    piston.set(Value.kForward);
  }
  public void pistonReverse(){
    piston.set(Value.kReverse);
  }
  public void intake(double power)
  {
    intake.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
