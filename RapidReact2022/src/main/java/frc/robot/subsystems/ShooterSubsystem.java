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
import frc.robot.constants.ShooterConstants.HoodConstants;
import frc.robot.subsystems.vision.VisionSystem;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax flyWheelRight, hoodMotor, feederMotor;
  private VisionSystem shooterVision;
  private RelativeEncoder hoodEncoder;
  private RelativeEncoder flyWheelEncoder; 
  private ShuffleboardLayout shooterLayout;
  private SparkMaxPIDController flyWheelPidController;
  private SparkMaxPIDController hoodPidController;
  private double highRPM = ShooterConstants.highRPM;
  private double highHood = HoodConstants.highHood;
  private double hoodSetPoint = 0, flyWheelSetPoint = 0;

  private boolean readyToShoot;

  // funny array to store flywheel rpm and hood position to index, index values represent feet, ex index 0 is 0ft
  private double[] flyWheelRPM = {highRPM,highRPM,highRPM,highRPM,highRPM,highRPM,2350,2500, 2600, 2550, 2775, 2700, 2900, 3025, 3150, highRPM,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM};
  private double[] hoodPosition= {highHood,highHood,highHood,highHood,highHood,highHood,19.2,17.0, 19.6, 21.3, 23.5, 22.8, 29.5, 26.8, 29.0,highHood,highHood,highHood,highHood,highHood,highHood,highHood};
  // 6(72) feet = 2350, 1.92
  // 7(84) feet = 2500, 17.0
  // 8(96) feet = 2600, 19.6
  // 9(108) feet = 2550, 21.3
  // 10(120) feet = 2775, 23.5
  // 11(132) feet = 2700, 22.8
  // 12(144) feet = 2900, 29.5
  // 13(156) feet = 3025, 26.8
  // 14(168) feet = 3150, 29.0
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

    

    hoodEncoder = hoodMotor.getEncoder(Type.kHallSensor, 42);
    hoodEncoder.setPosition(0);
    hoodSetPoint = 0;

    flyWheelEncoder = flyWheelRight.getEncoder(Type.kHallSensor, 42);
    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay);
    shooterLayout = driverTab.getLayout("Shooter", BuiltInLayouts.kGrid).withPosition(Constants.shooterIndex, 0).withSize(1, 5);
    shooterLayout.addBoolean("ReadyToShoot", this::getReadyToShoot).withSize(1, 1);
    shooterLayout.addNumber("Hood Setpoint", this::getHoodSetPoint).withSize(1, 1);
    shooterLayout.addNumber("Hood Pos", this::getHoodPos).withSize(1, 1);
    shooterLayout.addNumber("Fly Wheel RPM", this::getRPM).withSize(1, 1);
    shooterLayout.addNumber("Fly Wheel SetPoint", this::getFlywheelSetPoint).withSize(1, 1);
    shooterLayout.addNumber("Default RPM Flywheel", () -> ShooterConstants.defaultFlyWheelRPM).withSize(1, 1);
    


  }
  
  public void spinUpWheelRPM() {
    flyWheelPidController.setFF(ShooterConstants.kS / flyWheelSetPoint + ShooterConstants.kV);
    flyWheelPidController.setReference(flyWheelSetPoint, CANSparkMax.ControlType.kVelocity);
  
  }

  public void setFlyWheelPoint(double setPoint){
    flyWheelSetPoint = setPoint;
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

  public double getHoodSetPoint(){
    return this.hoodSetPoint;
  }

  public double getHoodPos(){
    return this.hoodEncoder.getPosition();
  }

  public void stopHood() {
    hoodMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
