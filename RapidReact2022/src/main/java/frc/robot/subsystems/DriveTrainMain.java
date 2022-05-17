// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.Controller;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.DriveConstants;

public class DriveTrainMain extends SubsystemBase {
  /**
   * Creates a new DriveTrainMain.
   */
  private MotorController leftMaster;
  private MotorController rightMaster;
  
  //private DigitalInput limit = new DigitalInput(ControlConstants.limitSwitch);
  //private DigitalInput BB1 = new DigitalInput(ControlConstants.BB1);
  //private DigitalInput BB2 = new DigitalInput(ControlConstants.BB2);

  Pose pose;
  private DifferentialDrive diffDrive;

  // Simulation
  // Create our feedforward gain constants (from the identification tool)
  static final double KvLinear = DriveConstants.kvVoltSecondsPerMeter;
  static final double KaLinear = DriveConstants.kaVoltSecondsSquaredPerMeter;
  static final double KvAngular = 1.5;
  static final double KaAngular = 0.3;

  // Create the simulation model of our drivetrain.
  private DifferentialDrivetrainSim m_driveSim;
  public DriveTrainMain() {
    m_driveSim = new DifferentialDrivetrainSim(
      // Create a linear system from our identification gains.
      LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
      DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
      DriveConstants.motorRotationsPerWheelRotation,  // 10.71:1 gearing reduction.
      DriveConstants.kTrackwidthMeters, // The track width is 0.616 meters.
      KitbotWheelSize.kSixInch.value, // The robot uses 3" radius wheels.
    
      // The standard deviations for measurement noise:
      // x and y:          0.001 m
      // heading:          0.001 rad
      // l and r velocity: 0.1   m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  }

  public DriveTrainMain(MotorController left, MotorController right) {
    leftMaster = left;
    rightMaster = right;
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
    leftMaster.set(maxOutput);
    rightMaster.set(maxOutput);
  }

  @Override
  public void simulationPeriodic() {

  }
}
