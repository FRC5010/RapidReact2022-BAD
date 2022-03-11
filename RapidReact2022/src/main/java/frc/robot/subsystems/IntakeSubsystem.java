// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;

public class IntakeSubsystem extends SubsystemBase {
  DoubleSolenoid intakePiston;
  ColorSensorV3 colorSensor;
  ColorMatch m_colorMatcher;
  CANSparkMax intakeMotor;
  
  public IntakeSubsystem(CANSparkMax intakeMotor, DoubleSolenoid intakePiston, ColorSensorV3 colorSensor) {
    this.intakeMotor = intakeMotor;
    this.colorSensor = colorSensor;
    this.intakePiston = intakePiston;
    this.m_colorMatcher = new ColorMatch();
    m_colorMatcher.addColorMatch(IndexerConstants.kRedTarget);
    m_colorMatcher.addColorMatch(IndexerConstants.kBlueTarget);
  }
  public Color getColor(){
    return m_colorMatcher.matchClosestColor(colorSensor.getColor()).color;
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
