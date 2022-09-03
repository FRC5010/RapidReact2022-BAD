/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import java.io.IOException;
import java.nio.file.Path;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Robot;
import frc.robot.FRC5010.Controller;
import frc.robot.FRC5010.GenericEncoder;
import frc.robot.FRC5010.GenericGyro;
import frc.robot.FRC5010.Impl.NavXGyro;
import frc.robot.FRC5010.Impl.RevEncoder;
import frc.robot.FRC5010.Impl.SimulatedEncoder;
import frc.robot.FRC5010.Impl.SimulatedGyro;
import frc.robot.commands.DriveTrainYEET;
import frc.robot.commands.Driving;
import frc.robot.commands.RamseteFollower;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.vision.VisionSystem;

/**
 * Add your docs here.
 */
public class Drive {
  public static DriveTrainMain driveTrain;

  public static Joystick driver;
  public static Controller driver2; 
  
  public static CANSparkMax lDrive1;
  //public static CANSparkMax lDrive2;
  public static CANSparkMax lDrive3;

  public static CANSparkMax rDrive1;
  //public static CANSparkMax rDrive2;
  public static CANSparkMax rDrive3;

  public static GenericEncoder lEncoder;
  public static GenericEncoder rEncoder;

  // These are our EncoderSim objects, which we will only use in
  // simulation. However, you do not need to comment out these
  // declarations when you are deploying code to the roboRIO.
  private static Encoder m_leftEncoder;
  private static Encoder m_rightEncoder;

  public static Pose robotPose;
  public GenericGyro gyro;
  
  public JoystickButton intakeAimButton;
  public JoystickButton shooterAimButton;

  public POVButton incThrottleFactor;
  public POVButton decThrottleFactor;
  public POVButton incSteerFactor;
  public POVButton decSteerFactor;

  public JoystickButton switchDirection;

  public JoystickButton intakeDriveButton;
  public JoystickButton autoNavButton;
  public JoystickButton driveYEET;

  public Drive(Joystick driver, VisionSystem shooterVision, Controller driver2) {
    init(driver, shooterVision, driver2);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    driver2.setLeftYAxis(driver2.createLeftYAxis().negate().deadzone(.075).cubed().limit(1));
    driver2.setRightXAxis(driver2.createRightXAxis().deadzone(.075).cubed().limit(1));
      // incThrottleFactor = new POVButton(driver, ControlConstants.incThrottleFactor);


  driver2.createUpPovButton().whenPressed(new InstantCommand(() -> DriveConstants.throttleFactor = Math.min(1,
        DriveConstants.throttleFactor + DriveConstants.drivingAdjustment)));

    // decThrottleFactor = new POVButton(driver, ControlConstants.decThrottleFactor);

  driver2.createDownPovButton().whenPressed(new InstantCommand(() -> DriveConstants.throttleFactor = Math.max(0,
        DriveConstants.throttleFactor - DriveConstants.drivingAdjustment)));

    // incSteerFactor = new POVButton(driver, ControlConstants.incSteerFactor);

  driver2.createRightPovButton().whenPressed(new InstantCommand(
        () -> DriveConstants.steerFactor = Math.min(1, DriveConstants.steerFactor + DriveConstants.drivingAdjustment)));

    // decSteerFactor = new POVButton(driver, ControlConstants.decSteerFactor);

  driver2.createLeftPovButton().whenPressed(new InstantCommand(
        () -> DriveConstants.steerFactor = Math.max(0, DriveConstants.steerFactor - DriveConstants.drivingAdjustment)));

    // driveYEET = new JoystickButton(driver, ControlConstants.driveYEET);
  driver2.createLeftBumper().whileHeld(new DriveTrainYEET(driveTrain, driver));

  }

  public static void setCurrentLimits(int currentLimit) {
    if (RobotBase.isReal()) {
      lDrive1.setSmartCurrentLimit(currentLimit);
      //lDrive2.setSmartCurrentLimit(currentLimit);
      lDrive3.setSmartCurrentLimit(currentLimit);
      rDrive1.setSmartCurrentLimit(currentLimit);
      //rDrive2.setSmartCurrentLimit(currentLimit);
      rDrive3.setSmartCurrentLimit(currentLimit);
    }
  }

  public void init(Joystick driver, VisionSystem shooterVision, Controller driver2) {
    this.driver = driver;
    this.driver2 = driver2; 
    // 10.71:1 gearbox on drivetrain as of 3/17/2022

    // Neos HAVE to be in brushless
    lDrive1 = new CANSparkMax(ControlConstants.leftDrive1M, MotorType.kBrushless);
    lDrive1.restoreFactoryDefaults();
    lDrive1.setInverted(false);
    rDrive1 = new CANSparkMax(ControlConstants.rightDrive1M, MotorType.kBrushless);
    rDrive1.restoreFactoryDefaults();
    rDrive1.setInverted(true);
    if (RobotBase.isReal()) {
      gyro = new NavXGyro(Port.kMXP);
      lDrive3 = new CANSparkMax(ControlConstants.leftDrive3M, MotorType.kBrushless);
      rDrive3 = new CANSparkMax(ControlConstants.rightDrive3M, MotorType.kBrushless);

      lDrive3.restoreFactoryDefaults();
      rDrive3.restoreFactoryDefaults();
      lDrive3.follow(lDrive1, false);

      rDrive3.follow(rDrive1, false);
      rDrive3.setInverted(false);

      rDrive1.setOpenLoopRampRate(0.5);
      lDrive1.setOpenLoopRampRate(0.5);

      RelativeEncoder leftEncoder = lDrive1.getEncoder();
      leftEncoder.setPositionConversionFactor(DriveConstants.leftDistanceConv);
      leftEncoder.setVelocityConversionFactor(DriveConstants.leftVelocityConv);
      lEncoder = new RevEncoder(leftEncoder);

      RelativeEncoder rightEncoder = rDrive1.getEncoder();
      rightEncoder.setPositionConversionFactor(DriveConstants.rightDistanceConv);
      rightEncoder.setVelocityConversionFactor(DriveConstants.rightVelocityConv);
      rEncoder = new RevEncoder(rightEncoder);

      setCurrentLimits(ControlConstants.driveTrainCurrentLimit);
    } else {
      gyro = new SimulatedGyro();
      m_leftEncoder = new Encoder(10,11);
      m_rightEncoder = new Encoder(12,13);
      m_leftEncoder.setDistancePerPulse(DriveConstants.leftDistanceConv);
      m_rightEncoder.setDistancePerPulse(DriveConstants.rightDistanceConv);
      lEncoder = new SimulatedEncoder(m_leftEncoder);
      rEncoder = new SimulatedEncoder(m_rightEncoder);
      initSimulation();
    }
    driveTrain = new DriveTrainMain(lDrive1, rDrive1);
    robotPose = new Pose(lEncoder, rEncoder, gyro);
    robotPose.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }
//Just sets up defalt commands (setUpDeftCom)
  public void setUpDeftCom() {
    driveTrain.setDefaultCommand(new Driving(driveTrain, driver, driver2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   */

  public static Command getAutonomousCommand(String path, boolean reset) {
    // Create a voltage constraint to ensure we don't accelerate too fast
    String trajectoryJSON = path;
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    RamseteCommand ramseteCommand = new RamseteFollower(trajectory, reset);

    Command result = ramseteCommand;

    // Run path following command, then stop at the end.
    return result;
  }

  public DriveTrainMain getDriveTrainMain() {
    return driveTrain;
  }

  public Pose getPose() {
    return robotPose;
  }
  
  // Simulation
  // Create our feedforward gain constants (from the identification tool)
  static final double KvLinear = DriveConstants.kvVoltSecondsPerMeter;
  static final double KaLinear = DriveConstants.kaVoltSecondsSquaredPerMeter;
  static final double KvAngular = 1.5;
  static final double KaAngular = 0.3;
  // Create the simulation model of our drivetrain.
  public static DifferentialDrivetrainSim m_driveSim;
  public void initSimulation() {
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
}
