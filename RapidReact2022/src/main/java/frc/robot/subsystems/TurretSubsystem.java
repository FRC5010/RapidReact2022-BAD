// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionSystem;

public class TurretSubsystem extends SubsystemBase {
  
private CANSparkMax turretMotor;
private VisionSystem shooterVision;

  public TurretSubsystem(CANSparkMax turretMotor, VisionSystem shooterVision) {
    this.turretMotor = turretMotor;
    this.shooterVision = shooterVision;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
