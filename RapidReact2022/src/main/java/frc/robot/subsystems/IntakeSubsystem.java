// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  DoubleSolenoid intakePiston;
  CANSparkMax intakeMotor;
  
  public IntakeSubsystem(CANSparkMax intakeMotor, DoubleSolenoid intakePiston) {
    this.intakeMotor = intakeMotor;
    this.intakePiston = intakePiston;
  }
  public void togglePiston(){
    intakePiston.toggle();
  }
  public void pistonOff(){
    intakePiston.set(DoubleSolenoid.Value.kOff);
  }
  public void retractIntake(){
    intakePiston.set(DoubleSolenoid.Value.kReverse);
  }
  public void deployIntake(){
    intakePiston.set(DoubleSolenoid.Value.kForward);
  }

  public boolean isIntakeDeployed(){
    return intakePiston.get().equals(DoubleSolenoid.Value.kForward);
  }

  public void setIntakePow(double pow){
    intakeMotor.set(pow);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
