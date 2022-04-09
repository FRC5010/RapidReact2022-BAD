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

private static boolean onTarget;
private double turretMovePow = 0;

private RelativeEncoder turretEncoder;


private ShuffleboardLayout turretLayout;

  public TurretSubsystem(CANSparkMax turretMotor, VisionSystem shooterVision) {
    this.turretMotor = turretMotor;
    this.shooterVision = shooterVision;
    this.turretEncoder = turretMotor.getEncoder(Type.kHallSensor, 42);
    ShuffleboardTab turretTab = Shuffleboard.getTab(ControlConstants.SBTabDiagnostics);
    turretLayout = turretTab.getLayout("Turret", BuiltInLayouts.kGrid).withPosition(Constants.turretIndex, 0).withSize(1, 5);
    turretLayout.addNumber("Turret Pos", this::getTurretPos).withSize(1, 1);
    turretLayout.addBoolean("Turret Is On Target", TurretSubsystem::getIsOnTarget).withSize(1, 1);
    turretLayout.addNumber("TurretPow", this::getTurretPow);

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
    double anglePow = pVal + dVal; // + ((1) * Math.signum(angle) * TurretConstants.kS);
    turretMovePow = 0;
    // if(Math.abs(angle) > TurretConstants.onTargetLowLimit){
    //   limit = Math.min(TurretConstants.limitPow, Math.max(anglePow, -TurretConstants.limitPow));  
    //   // limit = Math.signum(angle) * TurretConstants.kS;
    // }
    
    // double currPos = turretEncoder.getPosition();
    // if(currPos < TurretConstants.leftLimit || currPos > TurretConstants.rightLimit){
    //   limit = TurretConstants.kS * Math.signum(currPos);
    // }

    // try to keep the turret at the limit only when its trying to go to the limit in the same direction of the limit
    // ex target is right side of camera, and turret is at right limit
    double currPos = turretEncoder.getPosition();

    if(Math.abs(angle) > TurretConstants.onTargetLowLimit){
      turretMovePow = Math.min(TurretConstants.limitPow, Math.max(anglePow, -TurretConstants.limitPow));
    } else {
      turretMovePow = TurretConstants.kS/2 * Math.signum(angle);
    }
    
    if(currPos < TurretConstants.leftLimit){
      if(angle < 0){
        turretMovePow = TurretConstants.kS * Math.signum(currPos);
      }
    }else if(currPos > TurretConstants.rightLimit){
      if(angle > 0){
        turretMovePow = TurretConstants.kS * Math.signum(currPos);
      }
    } 

    
    turretMotor.set(turretMovePow);
    
  }

  public static boolean isOnTarget(double angle){
    onTarget = Math.abs(angle) <= TurretConstants.onTargetHighLimit;
    return onTarget;
  }

  public double getTurretPos(){
    return turretEncoder.getPosition();
  }

  public double getTurretPow(){
    return turretMovePow;
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
  public void setOnTarget(boolean target){
    onTarget = target;
  }
  public static boolean getIsOnTarget(){
    return onTarget;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
