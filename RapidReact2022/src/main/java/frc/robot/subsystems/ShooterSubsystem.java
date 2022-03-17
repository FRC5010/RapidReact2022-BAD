// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.FeederConstants;
import frc.robot.constants.ShooterConstants.HoodConstants;
import frc.robot.subsystems.vision.VisionSystem;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax flyWheelRight, hoodMotor, feederMotor;
  private VisionSystem shooterVision;
  private RelativeEncoder hoodEncoder;
  private RelativeEncoder flyWheelEncoder;
  private RelativeEncoder feederEncoder;
  private ShuffleboardLayout shooterLayout;
  private SparkMaxPIDController flyWheelPidController;
  private SparkMaxPIDController hoodPidController;
  private SparkMaxPIDController feederPidController;
  private double highRPM = ShooterConstants.highRPM;
  private double highHood = HoodConstants.highHood;
  private double hoodSetPoint = 0, flyWheelSetPoint = 0,feederSetPoint = 0;

  private boolean readyToShoot;

  // funny array to store flywheel rpm and hood position to index, index values represent feet, ex index 0 is 0ft
  //private double[] flyWheelRPM = {highRPM,highRPM,highRPM,highRPM,highRPM,highRPM,2100,2175, 2350, 2500, 2575, 2700, 2950, 3575, 4650, highRPM,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM};
  //private double[] hoodPosition= {highHood,highHood,highHood,highHood,highHood,highHood,21.2,22.1, 21.5, 23.4, 28.2, 27.4, 32.2, 37.9, 37.7,highHood,highHood,highHood,highHood,highHood,highHood,highHood};
  // 6(72) feet = 2100, 21.2
  // 7(84) feet = 2175, 22.1
  // 8(96) feet = 2350, 21.5
  // 9(108) feet = 2500, 23.4
  // 10(120) feet = 2575, 28.2
  // 11(132) feet = 2700, 27.4
  // 12(144) feet = 2950, 32.2
  // 13(156) feet = 3575, 37.8
  // 14(168) feet = 4650, 37.7
  // 15(180) feet =
  // 16(192) feet = 
  // 17(204) feet =

  // new array of 3/17/2022 with foam
  private double[] flyWheelRPM = {highRPM,highRPM, highRPM, highRPM ,2225,2250,2325,2425, 2500, 2600, 2650, 2800, 3150, 3150, 3150,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM};
  private double[] hoodPosition= {highHood,highHood,highHood,highHood,15.0,17.0,19.2,20.2, 22.9, 24.8, 28.2, 30.0, 37.0, 37.0, 37.0,highHood,highHood,highHood,highHood,highHood,highHood,highHood};
  // 4(48) feet = 2225, 15
  // 5(60) feet = 2250, 17
  // 6(72) feet = 2325, 19.2
  // 7(84) feet = 2425, 20.2
  // 8(96) feet = 2500, 22.9
  // 9(108) feet = 2600, 24.8
  // 10(120) feet = 2650, 28.2
  // 11(132) feet = 2800, 30.0
  // 12(144) feet = 3150, 37.0
  // 13(156) feet = 3150, 37.0
  // 14(168) feet = 3150, 37.0
  // 15(180) feet =
  // 16(192) feet = 
  // 17(204) feet =
  

  public ShooterSubsystem(CANSparkMax flyWheelRight, CANSparkMax hoodMotor, CANSparkMax feederMotor, VisionSystem shooterVision) {
    this.flyWheelRight = flyWheelRight;
    this.hoodMotor = hoodMotor;
    this.feederMotor = feederMotor;
    this.shooterVision = shooterVision;

    flyWheelPidController = flyWheelRight.getPIDController();
    flyWheelPidController.setP(ShooterConstants.kP);
    flyWheelPidController.setI(ShooterConstants.kI);
    flyWheelPidController.setD(ShooterConstants.kD);

    hoodPidController = hoodMotor.getPIDController();
    hoodPidController.setP(ShooterConstants.HoodConstants.kP);
    hoodPidController.setI(ShooterConstants.HoodConstants.kI);
    hoodPidController.setD(ShooterConstants.HoodConstants.kD);

    feederPidController = feederMotor.getPIDController();
    feederPidController.setP(ShooterConstants.FeederConstants.kP);
    feederPidController.setI(ShooterConstants.FeederConstants.kI);
    feederPidController.setD(ShooterConstants.FeederConstants.kD);

    

    hoodEncoder = hoodMotor.getEncoder(Type.kHallSensor, 42);
    hoodEncoder.setPosition(0);
    hoodSetPoint = 0;

    feederEncoder = feederMotor.getEncoder(Type.kHallSensor, 42);
    feederEncoder.setPosition(0);
    feederSetPoint = 0;

    flyWheelEncoder = flyWheelRight.getEncoder(Type.kHallSensor, 42);
    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay);
    shooterLayout = driverTab.getLayout("Shooter", BuiltInLayouts.kGrid).withPosition(Constants.shooterIndex, 0).withSize(1, 5);
    shooterLayout.addBoolean("ReadyToShoot", this::getReadyToShoot).withSize(1, 1);
    shooterLayout.addNumber("Hood Setpoint", this::getHoodSetPoint).withSize(1, 1);
    shooterLayout.addNumber("Hood Pos", this::getHoodPos).withSize(1, 1);
    shooterLayout.addNumber("Fly Wheel RPM", this::getRPM).withSize(1, 1);
    shooterLayout.addNumber("Fly Wheel SetPoint", this::getFlywheelSetPoint).withSize(1, 1);
    shooterLayout.addNumber("Default RPM Flywheel", () -> ShooterConstants.defaultFlyWheelRPM).withSize(1, 1);
    shooterLayout.addNumber("Feeder RPM", this::getFeederRPM);

  }
  
  public void spinUpWheelRPM() {
    flyWheelPidController.setFF(ShooterConstants.kS / flyWheelSetPoint + ShooterConstants.kV);
    flyWheelPidController.setReference(flyWheelSetPoint, CANSparkMax.ControlType.kVelocity);
  
  }

  public void setFlyWheelPoint(double setPoint){
    flyWheelSetPoint = setPoint + ShooterConstants.shotAdjustment;
  }

  // getters to check if hood or flywheel are in range
  public boolean getHoodReadyToShoot() {
    if (Math.abs(hoodSetPoint - hoodEncoder.getPosition()) < HoodConstants.shootingHoodTolerance){
      return true;
    }
    return false;
  }

  public void setHoodEncoder(double pos){
    hoodEncoder.setPosition(pos);
  }

  public void setHoodMotor(double pow){
    hoodMotor.set(pow);
  }

  public boolean getFlyWheelReadyToShoot() {
    double rpmRange = 25;
    if (readyToShoot) {
      rpmRange = 150;
    }
    return (flyWheelSetPoint - flyWheelRight.getEncoder().getVelocity()) < rpmRange;
  }

  public void determineIfReadyToShoot() {
    readyToShoot = getFlyWheelReadyToShoot() && getHoodReadyToShoot();
  }

  public boolean getReadyToShoot() {
    return readyToShoot;
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

  public boolean oldPidHood(){
    double currPos = hoodEncoder.getPosition();
    double power = -(hoodSetPoint - currPos) * 0.008;
    power = Math.min(0.2, Math.max(-0.2, power));
    //hoodMotor.set(power + 0.2);

    SmartDashboard.putNumber("powerHood", power);
    SmartDashboard.putNumber("currPosHood", currPos);
    SmartDashboard.putNumber("SetPointHood", hoodSetPoint);
    return Math.abs(power) < 0.01;
  }

  public boolean pidHood(){
    if (HoodConstants.hoodMaxPos > hoodSetPoint && hoodSetPoint > 0){
      hoodPidController.setReference(hoodSetPoint, CANSparkMax.ControlType.kPosition);
      return true;
    }
    return false;
  }



  public double hoodCalculations(double distanceCurrent) {
    // Linear Interpolation Calculations using indexes as feet ex index 0 is 0ft
    double hoodCurrent;
    double upperBound = Math.ceil(distanceCurrent);
    double lowerBound = Math.floor(distanceCurrent);

    if(upperBound <= hoodPosition.length - 1 && lowerBound >= 0){
      double hoodUpper = hoodPosition[(int) upperBound];
      double hoodLower = hoodPosition[(int) lowerBound];

    
      hoodCurrent = (distanceCurrent - lowerBound) * (hoodUpper - hoodLower) / (upperBound - lowerBound) + hoodLower;
    }else{
      hoodCurrent = hoodEncoder.getPosition();
    }
    
    return hoodCurrent;
  }

  public double flyWheelCalculations(double distanceCurrent) {
    // Linear Interpolation Calculations
    double flyWheelCurrent;
    double upperBound = Math.ceil(distanceCurrent);
    double lowerBound = Math.floor(distanceCurrent);

    if(upperBound <= hoodPosition.length - 1 && lowerBound >= 0){
      double flyWheelUpper = flyWheelRPM[(int) upperBound];
      double flyWheelLower = flyWheelRPM[(int) lowerBound];

    
      flyWheelCurrent = (distanceCurrent - lowerBound) * (flyWheelUpper - flyWheelLower) / (upperBound - lowerBound) + flyWheelLower;
    }else{
      flyWheelCurrent = 0;
    }
    return flyWheelCurrent;
  }

  public double getRPM(){
    return flyWheelEncoder.getVelocity();
  } 

  public double getFlywheelSetPoint(){
    return flyWheelSetPoint;
  }

  public void setHoodSetPoint(double setPoint){
    if (HoodConstants.hoodMaxPos > setPoint && setPoint > 0){
      this.hoodSetPoint = setPoint;
    }
  }

  public void runWithVelocityControl() {
    feederPidController.setFF(FeederConstants.kS / flyWheelSetPoint + FeederConstants.kV);
    feederPidController.setReference(feederSetPoint, CANSparkMax.ControlType.kVelocity);
  
  }

  public void setFlyFeederPoint(double setPoint){
    feederSetPoint = setPoint;
  }

  public double getHoodSetPoint(){
    return this.hoodSetPoint;
  }

  public double getHoodPos(){
    return this.hoodEncoder.getPosition();
  }

  public void stopHood() {
    hoodMotor.set(0);
  }

  public double getFeederRPM(){
    return feederEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
