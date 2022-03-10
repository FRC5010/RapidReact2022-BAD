// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new Indexer. */
  private CANSparkMax lowerMotor;
  private DigitalInput upperBB;

  private SparkMaxPIDController indexerPIDController;

  private ShuffleboardLayout indexerLayout;
  private double indexerSetPoint;
  
  public IndexerSubsystem(CANSparkMax lowerMotor, DigitalInput upperBB) {
    this.upperBB = upperBB;
    this.lowerMotor = lowerMotor;
    this.indexerPIDController = lowerMotor.getPIDController();


    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay);
    indexerLayout = driverTab.getLayout("Shooter", BuiltInLayouts.kGrid).withPosition(Constants.indexerIndex, 0).withSize(1, 5);
    indexerLayout.addBoolean("Cargo Present", this::getUpperBB);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinUpIndexerRPM() {
    indexerPIDController.setFF(IndexerConstants.kS / indexerSetPoint + IndexerConstants.kV);
    indexerPIDController.setReference(indexerSetPoint, CANSparkMax.ControlType.kVelocity);
  
  }

  public void setIndexerPoint(double setPoint){
    indexerSetPoint = setPoint;
  }

  public void setLowerIndexer(double speed){
    lowerMotor.set(speed);
  }

  public boolean isLowerIndexerRunning(){
    return Math.abs(lowerMotor.get()) > 0;
  }

  public boolean getUpperBB(){
    return !upperBB.get();
  }
}
