// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.vision.VisionSystem;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax flyWheelRight, hoodMotor, feederMotor;
  private VisionSystem shooterVision;
  private RelativeEncoder hoodEncoder;
  private RelativeEncoder flyWheelEncoder; 
  private ShuffleboardLayout shooterLayout;
  private SparkMaxPIDController pidController;

  private double hoodSetPoint, flyWheelSetPoint;

  private boolean readyToShoot;

  private double[] flyWheelRPM = {0,0,0,0};
  private double[] hoodPosition = {0,0,0,0};

  public ShooterSubsystem(CANSparkMax flyWheelRight, CANSparkMax hoodMotor, CANSparkMax feederMotor, VisionSystem shooterVision) {
    this.flyWheelRight = flyWheelRight;
    this.hoodMotor = hoodMotor;
    this.feederMotor = feederMotor;
    this.shooterVision = shooterVision;

    pidController = flyWheelRight.getPIDController();
    pidController.setP(ShooterConstants.flyWheelP);
    pidController.setI(ShooterConstants.flyWheelI);
    pidController.setD(ShooterConstants.flyWheelD);

    hoodEncoder = hoodMotor.getEncoder(Type.kHallSensor, 42);
    hoodSetPoint = hoodEncoder.getPosition();

    flyWheelEncoder = flyWheelRight.getEncoder(Type.kHallSensor, 42);
    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay);
    shooterLayout = driverTab.getLayout("Shooter", BuiltInLayouts.kGrid).withPosition(Constants.shooterIndex, 0).withSize(1, 5);
    shooterLayout.addNumber("Hood Pos", this::getHoodSetPoint).withSize(1, 1);
    shooterLayout.addNumber("Fly Wheel RPM", this::getRPM).withSize(1, 1);

  }
  
  public void spinUpWheelRPM() {
    pidController.setFF(ShooterConstants.kS / flyWheelSetPoint + ShooterConstants.kV);
    pidController.setReference(flyWheelSetPoint, CANSparkMax.ControlType.kVelocity);
  
  }

  public void setFlyWheelPoint(double setPoint){
    flyWheelSetPoint = setPoint;
  }

  // getters to check if hood or flywheel are in range
  public boolean getHoodReadyToShoot() {
    return true;
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

  public double getRPM(){
    return flyWheelEncoder.getVelocity();
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
