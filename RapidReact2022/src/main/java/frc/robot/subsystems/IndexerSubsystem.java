// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new Indexer. */
  private CANSparkMax indexMotor1;
  private CANSparkMax indexMotor2;
  private DigitalInput upperBB;

  public IndexerSubsystem(CANSparkMax indexMotor1, DigitalInput upperBB, CANSparkMax indexMotor2) {
    this.upperBB = upperBB;
    this.indexMotor1 = indexMotor1;
    this.indexMotor2 = indexMotor2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIndexM1(double speed){
    indexMotor1.set(speed);
  }

  public void setIndexM2(double speed){
    indexMotor2.set(speed);
  }

  public boolean getUpperBB(){
    return upperBB.get();
  }
}
