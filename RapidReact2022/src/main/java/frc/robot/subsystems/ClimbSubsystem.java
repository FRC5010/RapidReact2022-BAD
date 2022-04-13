// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ClimbSubsystem extends SubsystemBase {
  private CANSparkMax leftWinch;
  private CANSparkMax rightWinch;
  private DoubleSolenoid climbSolenoid;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;


  ShuffleboardLayout climbEncoderLayout;


  public ClimbSubsystem(CANSparkMax leftWinch, CANSparkMax rightWinch, DoubleSolenoid climbSolenoid) {
  this.leftWinch = leftWinch;
  this.rightWinch = rightWinch;

  this.climbSolenoid = climbSolenoid;

  this.leftEncoder = leftWinch.getEncoder(Type.kHallSensor, 42);
  this.rightEncoder = rightWinch.getEncoder(Type.kHallSensor, 42);
  leftEncoder.setPosition(0);
  rightEncoder.setPosition(0);

  }

  @Override
  public void periodic() {


    // This method will be called once per scheduler run
  }
  
  public void setLeftWinchSpeed(double speed) {
    leftWinch.set(speed);
  }
  public void setRightWinchSpeed(double speed) {
    rightWinch.set(speed);
  }

  public void zeroArms(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }
//should be called extendo arms
//by Truman
//this is my contribution to the programming team
 public void setClimbArmsVertical(){
  climbSolenoid.set(DoubleSolenoid.Value.kForward);
 }
  
 public void setClimbArmsDiagonal(){
  climbSolenoid.set(DoubleSolenoid.Value.kReverse);
 }

 public void toggleClimbArm() {
  climbSolenoid.toggle();
 }

 public boolean isClimbArmHorizontal(){
   return climbSolenoid.get().equals(DoubleSolenoid.Value.kForward);
 }

 public double getLeftEncoderValue(){
   return leftEncoder.getPosition();
 }

 public double getRightEncoderValue(){
   return rightEncoder.getPosition();
 }
 
}
