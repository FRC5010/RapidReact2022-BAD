// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;

public class ClimbSubsystem extends SubsystemBase {
  private CANSparkMax leftWinch;
  private CANSparkMax rightWinch;
  private CANSparkMax staticHooks;
  private DoubleSolenoid climbSolenoid;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder staticEncoder;

  ShuffleboardLayout climbEncoderLayout;


  public ClimbSubsystem(CANSparkMax leftWinch, CANSparkMax rightWinch, CANSparkMax staticHooks, DoubleSolenoid climbSolenoid) {
  this.leftWinch = leftWinch;
  this.rightWinch = rightWinch;
  this.staticHooks = staticHooks;
  this.climbSolenoid = climbSolenoid;

  this.leftEncoder = leftWinch.getEncoder(Type.kHallSensor, 42);
  this.rightEncoder = rightWinch.getEncoder(Type.kHallSensor, 42);
  this.staticEncoder = staticHooks.getEncoder(Type.kHallSensor, 42);


  int colIndex = 0;
  ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabClimbDisplay);
  climbEncoderLayout = driverTab.getLayout("Climb Encoders", BuiltInLayouts.kGrid).withPosition(colIndex, 0).withSize(3, 5);

  climbEncoderLayout.addBoolean("Is Climb Arms Horizontal", this::isClimbArmHorizontal);
  climbEncoderLayout.addNumber("Left Encoder Value", this::getLeftEncoderValue);
  climbEncoderLayout.addNumber("Right Encoder Value", this::getRightEncoderValue);
  climbEncoderLayout.addNumber("Static Encoder Value", this::getStaticEncoderValue);
  
  }

  @Override
  public void periodic() {


    // This method will be called once per scheduler run
  }

  public void setStaticHookSpeed(double speed) {
    staticHooks.set(speed);
  } 
  
  public void setLeftWinchSpeed(double speed) {
    leftWinch.set(speed);
  }
  public void setRightWinchSpeed(double speed) {
    rightWinch.set(speed);
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
   return climbSolenoid.get().equals(DoubleSolenoid.Value.kForward) ? true : false;
 }

 public double getLeftEncoderValue(){
   return leftEncoder.getPosition();
 }

 public double getRightEncoderValue(){
   return rightEncoder.getPosition();
 }

 public double getStaticEncoderValue(){
   return staticEncoder.getPosition();
 }
}
