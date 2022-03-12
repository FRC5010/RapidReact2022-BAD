// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.ControlConstants;

public class VerticalIndexerSubsystem extends SubsystemBase {
  CANSparkMax upperMotor;
  DigitalInput upperBB;
  private ShuffleboardLayout indexerLayout;

  public VerticalIndexerSubsystem(CANSparkMax upperMotor, DigitalInput upperBB) {
    this.upperMotor = upperMotor;
    this.upperBB = upperBB;

    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay);
    indexerLayout = driverTab.getLayout("Shooter", BuiltInLayouts.kGrid).withPosition(Constants.indexerIndex, 0).withSize(1, 5);
    indexerLayout.addBoolean("Upper Cargo Present", this::getUpperBB);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVerticalIndexer(double speed){
    upperMotor.set(speed);
  }

  public boolean getUpperBB(){
    return !upperBB.get();
  }
}
