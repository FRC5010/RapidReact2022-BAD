// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private CANSparkMax leftWinch;
  private CANSparkMax rightWinch;
  private CANSparkMax staticHooks;
  private DoubleSolenoid climbSolenoid;


  public ClimbSubsystem(CANSparkMax leftWinch, CANSparkMax rightWinch, CANSparkMax staticHooks, DoubleSolenoid climbSolenoid) {
  this.leftWinch = leftWinch;
  this.rightWinch = rightWinch;
  this.staticHooks = staticHooks;
  this.climbSolenoid = climbSolenoid;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setStaticHookSpeed(double speed) {

    staticHooks.set(speed);
  } 
  
  public void leftWinch(double speed) {

    leftWinch.set(speed);
  }
  public void rightWinch(double speed) {

    rightWinch.set(speed);
  }

 public void armUp(){

  climbSolenoid.set(DoubleSolenoid.Value.kForward);
 }
  
 public void armDown(){

  climbSolenoid.set(DoubleSolenoid.Value.kReverse);
 }
}
