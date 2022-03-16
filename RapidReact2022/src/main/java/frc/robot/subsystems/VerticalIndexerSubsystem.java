// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IndexerConstants;

public class VerticalIndexerSubsystem extends SubsystemBase {
  CANSparkMax verticalShortMotor;
  DigitalInput upperBB;
  private ShuffleboardLayout indexerLayout;
  private CANSparkMax verticalLongMotor;
  private SparkMaxPIDController longPIDController;
  private SparkMaxPIDController shortPIDController;

  private double indexerSetPoint;
  private RelativeEncoder longEncoder;
  private RelativeEncoder shortEncoder;

  public VerticalIndexerSubsystem(CANSparkMax verticalShortMotor, CANSparkMax verticalLongMotor, DigitalInput upperBB) {
    this.verticalShortMotor = verticalShortMotor;
    this.verticalLongMotor = verticalLongMotor;
    this.upperBB = upperBB;

    this.longPIDController = verticalShortMotor.getPIDController();
    longPIDController.setP(IndexerConstants.VerticalLong.kP);
    longPIDController.setI(IndexerConstants.VerticalLong.kI);
    longPIDController.setD(IndexerConstants.VerticalLong.kD);

    this.shortPIDController = verticalLongMotor.getPIDController();
    shortPIDController.setP(IndexerConstants.VerticalShort.kP);
    shortPIDController.setI(IndexerConstants.VerticalShort.kI);
    shortPIDController.setD(IndexerConstants.VerticalShort.kD);



    this.longEncoder = verticalLongMotor.getEncoder();
    this.shortEncoder = verticalShortMotor.getEncoder();


    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDiagnostics);
    indexerLayout = driverTab.getLayout("Indexers", BuiltInLayouts.kGrid).withPosition(Constants.indexerIndex, 0).withSize(1, 5);
    indexerLayout.addBoolean("Upper Cargo Present", this::getUpperBB);
    indexerLayout.addNumber("Vertical Long RPM", this::getVerticalLongRPM);
    indexerLayout.addNumber("Vertical Short RPM", this::getVerticalShortRPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runWithVelocityControl() {
    shortPIDController.setFF(IndexerConstants.VerticalShort.kS / indexerSetPoint + IndexerConstants.VerticalShort.kV);
    shortPIDController.setReference(indexerSetPoint, CANSparkMax.ControlType.kVelocity);

    longPIDController.setFF(IndexerConstants.VerticalLong.kS / indexerSetPoint + IndexerConstants.VerticalLong.kV);
    longPIDController.setReference(indexerSetPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void setVerticalIndexerPoint(double setPoint){
    indexerSetPoint = setPoint;
  }

  public void setVerticalIndexer(double speed){
    verticalShortMotor.set(speed);
    verticalLongMotor.set(speed);
  }

  public boolean getUpperBB(){
    return !upperBB.get();
  }

  public double getVerticalLongRPM(){
    return longEncoder.getVelocity();
  }

  public double getVerticalShortRPM(){
    return shortEncoder.getVelocity();
  }
}
