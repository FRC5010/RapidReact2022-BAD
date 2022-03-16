// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

public class DiagonalIndexerSubsystem extends SubsystemBase {
  /** Creates a new Indexer. */
  private CANSparkMax diagonalLowerMotor;
  private CANSparkMax diagonalUpperMotor;
  private DigitalInput lowerBB;

  private SparkMaxPIDController lowerPIDController;
  private SparkMaxPIDController upperPIDController;

  private RelativeEncoder upperEncoder;
  private RelativeEncoder lowerEncoder;

  private ShuffleboardLayout indexerLayout;
  private double indexerSetPoint;
  
  public DiagonalIndexerSubsystem(CANSparkMax diagonalLowerMotor, CANSparkMax diagonalUpperMotor, DigitalInput upperBB) {
    this.lowerBB = upperBB;
    this.diagonalLowerMotor = diagonalLowerMotor;
    this.diagonalUpperMotor = diagonalUpperMotor;
    this.lowerPIDController = diagonalLowerMotor.getPIDController();
    lowerPIDController.setP(IndexerConstants.DiagonalLower.kP);
    lowerPIDController.setI(IndexerConstants.DiagonalLower.kI);
    lowerPIDController.setD(IndexerConstants.DiagonalLower.kD);


    this.upperPIDController = diagonalUpperMotor.getPIDController();
    upperPIDController.setP(IndexerConstants.DiagonalUpper.kP);
    upperPIDController.setI(IndexerConstants.DiagonalUpper.kI);
    upperPIDController.setD(IndexerConstants.DiagonalUpper.kD);

    upperEncoder = diagonalUpperMotor.getEncoder();
    lowerEncoder = diagonalLowerMotor.getEncoder();



    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDiagnostics);
    indexerLayout = driverTab.getLayout("Indexers", BuiltInLayouts.kGrid).withPosition(Constants.indexerIndex, 0).withSize(1, 5);
    indexerLayout.addBoolean("Lower Cargo Present", this::getLowerBB);
    indexerLayout.addNumber("Diagonal Lower RPM", this::getLowerRPM);
    indexerLayout.addNumber("Diagonal Upper RPM", this::getUpperRPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runWithVelocityControl() {
    lowerPIDController.setFF(IndexerConstants.DiagonalLower.kS / indexerSetPoint + IndexerConstants.DiagonalLower.kV);
    lowerPIDController.setReference(indexerSetPoint, CANSparkMax.ControlType.kVelocity);

    upperPIDController.setFF(IndexerConstants.DiagonalUpper.kS / indexerSetPoint + IndexerConstants.DiagonalUpper.kV);
    upperPIDController.setReference(indexerSetPoint, CANSparkMax.ControlType.kVelocity);
    
  
  }

  public void setDiagonalIndexerPoint(double setPoint){
    indexerSetPoint = setPoint;
  }

  public void setDiagonalIndexer(double speed){
    diagonalLowerMotor.set(speed);
    diagonalUpperMotor.set(speed);
  }

  public boolean isDiagonalIndexerRunning(){
    return Math.abs(diagonalLowerMotor.get()) > 0;
  }

  public boolean getLowerBB(){
    return !lowerBB.get();
  }

  public double getUpperRPM(){
    return upperEncoder.getVelocity();
  }
  public double getLowerRPM(){
    return lowerEncoder.getVelocity();
  }
}
