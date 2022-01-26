// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SetPipeline;
import frc.robot.commands.auto.GalacticSearch;
import frc.robot.commands.auto.HubToBall2;
import frc.robot.commands.auto.HubToBall3;
import frc.robot.commands.auto.LowerCargoToHub;
import frc.robot.commands.auto.ManyBallAuto;
import frc.robot.constants.ControlConstants;
import frc.robot.mechanisms.Drive;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.vision.VisionLimeLight;
import frc.robot.subsystems.vision.VisionLimeLightH;
import frc.robot.subsystems.vision.VisionSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Joystick driver;
  private Joystick operator;
  private SendableChooser<Command> command = new SendableChooser<>();
  private SendableChooser<Command> teamColor = new SendableChooser<>();

  private VisionLimeLight shooterVision;
  private VisionSystem intakeVision;

  private CameraSubsystem cameraSubsystem;

  private Drive drive; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driver = new Joystick(ControlConstants.driverJoystick);
    operator = new Joystick(ControlConstants.operatorJoystick);


    shooterVision = new VisionLimeLight("limelight-shooter", 19.25, 14.562694, 102.559, ControlConstants.shooterVisionColumn);
    intakeVision = new VisionLimeLightH("limelight-intake", 24, -5, 6, ControlConstants.shooterVisionColumn);

    drive = new Drive(driver,shooterVision);
    
    cameraSubsystem = new CameraSubsystem(operator);

    command.addOption("LowerCargoToHub", new LowerCargoToHub());
    command.addOption("HubBall2", new HubToBall2());
    command.addOption("HubBall3", new HubToBall3());
    command.addOption("ManyBall", new ManyBallAuto());
    command.addOption("Galactic Search", new GalacticSearch(drive.getDriveTrainMain(), shooterVision, drive.getPose()));

    teamColor.setDefaultOption("VTargets", new InstantCommand(() -> shooterVision.setPipeline(2)));
    teamColor.addOption("Red", new InstantCommand(() -> shooterVision.setPipeline(1)));
    teamColor.addOption("Blue", new InstantCommand(() -> shooterVision.setPipeline(0)));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Command red = new InstantCommand(() -> shooterVision.setPipeline(1));
    Command blue = new InstantCommand(() -> shooterVision.setPipeline(0));
    Command vTargets = new InstantCommand(() -> shooterVision.setPipeline(2));
    // this adds auto selections in SmartDashboard
    Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay).getLayout("Auto", BuiltInLayouts.kList)
        .withPosition(ControlConstants.autoColumn, 0).withSize(3, 1).add("Choose an Auto Mode", command)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);

    SmartDashboard.putData("red", new SetPipeline(1, shooterVision));
    SmartDashboard.putData("blue", new SetPipeline(0, shooterVision));
    SmartDashboard.putData("default", new SetPipeline(2, shooterVision));



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return command.getSelected();
  }
}
