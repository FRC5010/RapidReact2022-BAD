// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.vision.VisionSystem;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax flyWheelRight, hoodMotor, feederMotor;
  private VisionSystem shooterVision;
  private RelativeEncoder hoodEncoder;
  private ShuffleboardLayout shooterLayout;

  private double hoodSetPoint;

  private double[] flyWheelRPM = {0,0,0,0};
  private double[] hoodPosition = {0,0,0,0};

  public ShooterSubsystem(CANSparkMax flyWheelRight, CANSparkMax hoodMotor, CANSparkMax feederMotor, VisionSystem shooterVision) {
    this.flyWheelRight = flyWheelRight;
    this.hoodMotor = hoodMotor;
    this.feederMotor = feederMotor;
    this.shooterVision = shooterVision;

    hoodEncoder = hoodMotor.getEncoder(Type.kHallSensor, 42);
    hoodSetPoint = hoodEncoder.getPosition();
    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabVisionDisplay);
    shooterLayout = driverTab.getLayout("Shooter", BuiltInLayouts.kGrid).withPosition(Constants.shooterIndex, 0).withSize(1, 5);
    shooterLayout.addNumber("Hood Pos", this::getHoodSetPoint).withSize(1, 1);
  }

  public void spinFlyWheel(double speed){
    flyWheelRight.set(speed);
  }

  public void spinFeeder(double speed){
    feederMotor.set(speed);
  }

  public void spinHood(double speed){
    hoodMotor.set(speed);
  }

  public boolean pidHood(){
    double currPos = hoodEncoder.getPosition();
    double power = -(hoodSetPoint - currPos) * 0.008;
    power = Math.min(0.2, Math.max(-0.2, power));
    //hoodMotor.set(power + 0.2);

    SmartDashboard.putNumber("powerHood", power);
    SmartDashboard.putNumber("currPosHood", currPos);
    SmartDashboard.putNumber("SetPointHood", hoodSetPoint);
    return Math.abs(power) < 0.01;
  }
  public double hoodCalculations(double distanceCurrent) {
    // Linear Interpolation Calculations using indexes as feet ex index 0 is 0ft
    double upperBound = Math.ceil(distanceCurrent);
    double lowerBound = Math.floor(distanceCurrent);

    double hoodUpper = hoodPosition[(int) upperBound];
    double hoodLower = hoodPosition[(int) lowerBound];

    
    double hoodCurrent = (distanceCurrent - lowerBound) * (hoodUpper - hoodLower) / (upperBound - lowerBound) + hoodLower;

    return hoodCurrent;
  }

  public double flyWheelCalculations(double distanceCurrent) {
    // Linear Interpolation Calculations
    double upperBound = Math.ceil(distanceCurrent);
    double lowerBound = Math.floor(distanceCurrent);

    double flyWheelUpper = flyWheelRPM[(int) upperBound];
    double flyWheelLower = flyWheelRPM[(int) lowerBound];

    
    double flyWheelCurrent = (distanceCurrent - lowerBound) * (flyWheelUpper - flyWheelLower) / (upperBound - lowerBound) + flyWheelLower;

    return flyWheelCurrent;
  }



  public void setHoodSetPoint(double setPoint){
    this.hoodSetPoint = setPoint;
  }

  public double getHoodSetPoint(){
    return this.hoodSetPoint;
  }

  public void stopHood() {
    hoodMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
