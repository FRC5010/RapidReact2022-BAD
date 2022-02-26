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
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.TurretConstants;
import frc.robot.subsystems.vision.VisionSystem;

public class TurretSubsystem extends SubsystemBase {
  
private CANSparkMax turretMotor;
private VisionSystem shooterVision;

private boolean onTarget;

private RelativeEncoder turretEncoder;


private ShuffleboardLayout turretLayout;

  public TurretSubsystem(CANSparkMax turretMotor, VisionSystem shooterVision) {
    this.turretMotor = turretMotor;
    this.shooterVision = shooterVision;
    this.turretEncoder = turretMotor.getEncoder(Type.kHallSensor, 42);
    ShuffleboardTab turretTab = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay);
    turretLayout = turretTab.getLayout("Turret", BuiltInLayouts.kGrid).withPosition(Constants.turretIndex, 0).withSize(1, 5);
    turretLayout.addNumber("Turret Pos", this::getTurretPos).withSize(1, 1);
    turretLayout.addBoolean("Turret Is On Target", this::getIsOnTarget).withSize(1, 1);

    SmartDashboard.putNumber("TurretP", TurretConstants.kPVision);
    SmartDashboard.putNumber("TurretD", TurretConstants.kDVision);

  }

  public void turnTurret(double speed){
    turretMotor.set(speed);
  }

  public void centerTurret(){
    double posPow = turretEncoder.getPosition() * TurretConstants.kPEncoder;
    double limit = Math.min(TurretConstants.limitPow, Math.max(posPow, -TurretConstants.limitPow));
    turretMotor.set(-limit);
  }

  public void angleTurret(double angle, double lastTime, double lastAngle){
    double p = SmartDashboard.getNumber("TurretP", TurretConstants.kPVision);
    double d = SmartDashboard.getNumber("TurretD", TurretConstants.kDVision);
    double pVal = angle * p;
    double dVal = d * ((angle - lastAngle) / ((System.currentTimeMillis() - lastTime) * 1000));
    double anglePow = pVal + dVal;
    double limit = Math.min(TurretConstants.limitPow, Math.max(anglePow, -TurretConstants.limitPow));

    double currPos = turretEncoder.getPosition();
    if(currPos < TurretConstants.leftLimit || currPos > TurretConstants.rightLimit){
      limit = 0;
    }
    SmartDashboard.putNumber("TurretPow", -limit);
    turretMotor.set(-limit);
  }

  public boolean isOnTarget(double angle){
    onTarget = Math.abs(angle) <= 1;
    return onTarget;
  }

  public double getTurretPos(){
    return turretEncoder.getPosition();
  }

  public void zeroTurret(){
    turretEncoder.setPosition(0);
  }
  public boolean isAtRightLimit(){
    return this.getTurretPos() > TurretConstants.rightLimit; 
  }
  public boolean isAtLeftLimit(){
    return this.getTurretPos() < TurretConstants.leftLimit; 
  }
  public void setOnTarget(boolean onTarget){
    this.onTarget = onTarget;
  }

  public boolean getIsOnTarget(){
    return onTarget;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
