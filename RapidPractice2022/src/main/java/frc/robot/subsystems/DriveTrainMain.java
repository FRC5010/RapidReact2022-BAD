// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.commands.ComboDrive;
import frc.robot.commands.CurvatureDrive;
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
import frc.robot.commands.Driving;
import frc.robot.constants.DriveConstants;

public class DriveTrainMain extends SubsystemBase {
  /**
   * Creates a new DriveTrainMain.
   */
  private MotorController leftMaster;
  private MotorController rightMaster;
  
  private DigitalInput limit = new DigitalInput(ControlConstants.limitSwitch);
  private DigitalInput BB1 = new DigitalInput(ControlConstants.BB1);
  private DigitalInput BB2 = new DigitalInput(ControlConstants.BB2);

  Pose pose;
  private DifferentialDrive diffDrive;

  public DriveTrainMain(MotorController left, MotorController right, Joystick driver, Pose pose) {
    leftMaster = left;
    rightMaster = right;
    this.pose = pose;
    diffDrive = new DifferentialDrive(left, right);

    ShuffleboardLayout layout = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay)
      .getLayout("Driver", BuiltInLayouts.kList)
      .withPosition(ControlConstants.driverColumn, 2)
      .withSize(2, 5);
    layout.addNumber("Throttle Factor", this::getThrottleFactorDisplay)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("Max", 100, "Min", 0))
      .withPosition(ControlConstants.driverColumn, 3);
    layout.addNumber("Steer Factor", this::geSteerFactorDisplay)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("Max", 100, "Min", 0))
      .withPosition(ControlConstants.driverColumn, 5);
      layout.addBoolean("Drive Direction", this::getDriveDirection)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withPosition(ControlConstants.driverColumn, 1);

  //  pdp = new PowerDistributionPanel();
    setDefaultCommand(new ComboDrive(this, driver));
   // setDefaultCommand(new FlickStick(this, driver, pose));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("limit", limit.get());
    SmartDashboard.putBoolean("BB1", BB1.get());
    SmartDashboard.putBoolean("BB2", BB2.get());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
  }

  public double getThrottleFactorDisplay() {
    return DriveConstants.throttleFactor * 100;
  }
  public double geSteerFactorDisplay() {
    return DriveConstants.steerFactor * 100;
  }

  public boolean getDriveDirection(){
    return DriveConstants.driveInversion == 1;
  }
  //Driver means it's being affected by sensitivity
  // Non driver is full power
  public void driverArcadeDrive(double throttle, double steer) {
    steer *= DriveConstants.steerFactor;
    throttle *= DriveConstants.throttleFactor * DriveConstants.driveInversion;
    diffDrive.arcadeDrive(throttle, steer);
  }
  public void arcadeDrive(double throttle, double steer) {
    throttle *= DriveConstants.driveInversion;
    diffDrive.arcadeDrive(throttle, steer);
  }
  public void driverCurvatureDrive(double throttle, double steer){
    steer *= DriveConstants.steerFactor;
    throttle *= DriveConstants.throttleFactor * DriveConstants.driveInversion;
    diffDrive.curvatureDrive(throttle, steer, true);
  }
  public void driverCurvatureDrive(double throttle, double steer, boolean turnStop){
    steer *= DriveConstants.steerFactor;
    throttle *= DriveConstants.throttleFactor * DriveConstants.driveInversion;
    diffDrive.curvatureDrive(throttle, steer, turnStop);
  }
  public static double scaleInputs(double input) {
    if (input > -.1 && input < .1) {
      return 0.0;
    }
    if (input > 1) {
      return 1;
    }
    if (input < -1) {
      return -1;
    }
    
    return Math.pow(input, 3);

  }

  public void setMaxOutput(double maxOutput) {
    leftMaster.set(maxOutput);
    rightMaster.set(maxOutput);
  }

  public Pose getPose(){
    return pose;
  }
}
