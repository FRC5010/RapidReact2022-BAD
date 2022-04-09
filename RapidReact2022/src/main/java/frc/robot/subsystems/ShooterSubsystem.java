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
  private boolean hoodCalibrated = false; 
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
    shooterLayout.addNumber("Shot adjustment RPM", this::getShotAdjustment);
    shooterLayout.addBoolean("Hood Calibrated", this::getHoodCalibrated);
  }
  
  public void runFlyWheelWithVelocityControl() {
    flyWheelPidController.setFF(FeederConstants.kS / flyWheelSetPoint + FeederConstants.kV);
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

  public double getShotAdjustment(){
    return ShooterConstants.shotAdjustment;
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
      rpmRange = 75;
    }
    return (flyWheelSetPoint - flyWheelRight.getEncoder().getVelocity()) < rpmRange;
  }

  public double getFlyWheelPercentRPM(){
    return Math.abs(DriveTrainMain.minMaxOne(flyWheelRight.getEncoder().getVelocity() / flyWheelSetPoint));
    //return Math.max(Math.min(flyWheelSetPoint / flyWheelRight.getEncoder().getVelocity(), 1), 0);
  }

  public void determineIfReadyToShoot() {
    readyToShoot = getFlyWheelReadyToShoot() && getHoodReadyToShoot();
  }

  public void cancelReadyToShoot(){
    readyToShoot = false; 
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
    if (HoodConstants.hoodMaxPos > hoodSetPoint && hoodSetPoint > 0 && getHoodCalibrated()){
      hoodPidController.setReference(hoodSetPoint, CANSparkMax.ControlType.kPosition);
      return true;
    }
    return false;
  }

  public boolean getHoodCalibrated(){
    return hoodCalibrated;
  }

  public void setHoodCalibrated(boolean hoodCalibrated){
    this.hoodCalibrated = hoodCalibrated;
  }


  public double hoodCalculations(double distanceCurrent) {
    // Linear Interpolation Calculations using indexes as feet ex index 0 is 0ft
    double hoodCurrent;
    double upperBound = Math.ceil(distanceCurrent);
    double lowerBound = Math.floor(distanceCurrent);

    if(upperBound <= ShooterConstants.hoodPosition.length - 1 && lowerBound >= 0){
      double hoodUpper = ShooterConstants.hoodPosition[(int) upperBound];
      double hoodLower = ShooterConstants.hoodPosition[(int) lowerBound];

    
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

    if(upperBound <= ShooterConstants.hoodPosition.length - 1 && lowerBound >= 0){
      double flyWheelUpper = ShooterConstants.flyWheelRPM[(int) upperBound];
      double flyWheelLower = ShooterConstants.flyWheelRPM[(int) lowerBound];

    
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

  public void runFeederWheelWithVelocityControl() {
    feederPidController.setFF(FeederConstants.kS / feederSetPoint + FeederConstants.kV);
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
