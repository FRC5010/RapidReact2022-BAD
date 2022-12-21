// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrainMain.
   */
  private MotorController leftLeader;
  private MotorController rightLeader;

  //private DigitalInput limit = new DigitalInput(ControlConstants.limitSwitch);
  //private DigitalInput BB1 = new DigitalInput(ControlConstants.BB1);
  //private DigitalInput BB2 = new DigitalInput(ControlConstants.BB2);

  private DifferentialDrive diffDrive;

  public DriveTrain(MotorController left, MotorController right) {
    leftLeader = left;
    rightLeader = right;
    diffDrive = new DifferentialDrive(left, right);
  }

  public void init() {
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
  }

  @Override
  public void periodic() {
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.set(leftVolts/RobotController.getBatteryVoltage());
    rightLeader.set(rightVolts/RobotController.getBatteryVoltage());
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

    steer = driverModSteer(steer, throttle);
    throttle = driverModThrottle(throttle);

    //System.out.println(throttle);
    diffDrive.arcadeDrive(throttle, steer);
  }

  public void arcadeDrive(double throttle, double steer) {
    throttle *= DriveConstants.driveInversion;
    diffDrive.arcadeDrive(throttle, steer);
  }
  public void driverCurvatureDrive(double throttle, double steer){
    steer = driverModSteer(steer, throttle);
    throttle = driverModThrottle(throttle);

    diffDrive.curvatureDrive(throttle, steer, true);
  }
  public void driverCurvatureDrive(double throttle, double steer, boolean turnStop){
    steer = driverModSteer(steer, throttle);
    throttle = driverModThrottle(throttle);

    diffDrive.curvatureDrive(throttle, steer, turnStop);
  }

  public double driverModSteer(double steer, double throttle){
    return minMaxOne((deadzone(steer) * DriveConstants.steerFactor) + (Math.abs(steer) > 0 && Math.abs(throttle) == 0 ? (Math.signum(steer)) * DriveConstants.ksVolts : 0));
  }

  public double driverModThrottle(double throttle){
    return minMaxOne((deadzone(throttle) * DriveConstants.throttleFactor * DriveConstants.driveInversion) + ((Math.signum(throttle)) * DriveConstants.ksVolts));
  }

  public static double scaleInputs(double input) {
    input = deadzone(input);
    return Math.pow(input, 3);

  }

  public static double deadzone(double input) {
    if (input > -.075 && input < .075) {
      return 0.0;
    }
    input = minMaxOne(input);
    
    return input;

  }

  public static double minMaxOne(double input){
    if (input > 1) {
      return 1;
    }
    if (input < -1) {
      return -1;
    }
    
    return input;
  }

  public void setMaxOutput(double maxOutput) {
    leftLeader.set(maxOutput);
    rightLeader.set(maxOutput);
  }
}
