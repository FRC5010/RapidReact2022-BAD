// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.FRC5010.Controller;
import frc.robot.FRC5010.VisionSystem;
import frc.robot.FRC5010.Vision.VisionLimeLight;
import frc.robot.FRC5010.Vision.VisionLimeLightSim;
import frc.robot.commands.DefaultLed;
import frc.robot.commands.LedBlink;
import frc.robot.commands.LedColor;
import frc.robot.commands.LedRainbow;
import frc.robot.commands.SetPipeline;
import frc.robot.commands.SnapshotCmd;
import frc.robot.commands.Timer;
import frc.robot.commands.auto.ExtendingShortLeftTurn;
import frc.robot.commands.auto.ExtendingTerminalBall;
import frc.robot.commands.auto.ExtendingThreeBall;
import frc.robot.commands.auto.FenderTwoBall;
import frc.robot.commands.auto.TarmacToBall2;
import frc.robot.commands.auto.TarmacTwoBall;
import frc.robot.commands.auto.pathing.LowerBall1ToBall2;
import frc.robot.commands.auto.pathing.LowerBall2ToTerminal;
import frc.robot.commands.auto.pathing.LowerBall2ToTerminalA2Path;
import frc.robot.commands.auto.pathing.LowerTarmacToBall1;
import frc.robot.commands.auto.pathing.SingleCargoPath;
import frc.robot.commands.auto.pathing.TarmacToBall2Path;
import frc.robot.commands.auto.pathing.TerminalDriveBack;
import frc.robot.commands.auto.pathing.TerminalDriveBackA2;
import frc.robot.commands.auto.pathing.TurnBotLeft;
import frc.robot.commands.auto.pathing.UpperTarmacToBall;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.mechanisms.Climb;
import frc.robot.mechanisms.Drive;
import frc.robot.mechanisms.Transport;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.LedSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Joystick driver;

  private Joystick operator;

  private Controller driver2;
  private Controller operator2;

  private SendableChooser<Command> command = new SendableChooser<>();
  private SendableChooser<Command> teamColor = new SendableChooser<>();

  private VisionSystem shooterVision;
  private Transport transport;
  private Climb climb;

  private CameraSubsystem cameraSubsystem;
  private LedSubsystem ledSubsystem;

  private Drive drive;

  private JoystickButton toggleLL;

  private JoystickButton takeSnapshot;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  public static final Mechanism2d mech2d = new Mechanism2d(60, 60);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driver = new Joystick(ControlConstants.driverJoystick);
    operator = new Joystick(ControlConstants.operatorJoystick);

    driver2 = new Controller(Controller.JoystickPorts.ZERO.ordinal());
    operator2 = new Controller(Controller.JoystickPorts.ONE.ordinal());

    initRealOrSim();
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Mech Sim", mech2d);
    drive = new Drive(driver, shooterVision, driver2);
    transport = new Transport(operator, driver, shooterVision, driver2, operator2);
    if (RobotBase.isReal()) {
      climb = new Climb(driver, operator, transport, driver2, operator2);


      teamColor.setDefaultOption("VTargets", new InstantCommand(() -> shooterVision.setPipeline(2)));
      teamColor.addOption("Red", new InstantCommand(() -> shooterVision.setPipeline(1)));
      teamColor.addOption("Blue", new InstantCommand(() -> shooterVision.setPipeline(0)));
    }
    initAutoCommands();
    // Configure the button bindings
    configureButtonBindings();
  }

  private void initRealOrSim() {
    if (RobotBase.isReal()) {
      // the 20 degrees as of march 15 2022,
      shooterVision = new VisionLimeLight("limelight-shooter", 36.5, 37.0, 102.559,
          ControlConstants.shooterVisionColumn, "drive");

      ledSubsystem = new LedSubsystem(0, 62);
      SmartDashboard.putData("Leds Orange", new LedColor(255, 25, 0, ledSubsystem));
      SmartDashboard.putData("Leds Green", new LedColor(0, 255, 0, ledSubsystem));
      SmartDashboard.putData("Leds Off", new LedColor(0, 0, 0, ledSubsystem));
      SmartDashboard.putData("Led Blink Blue", new LedBlink(0, 0, 255, 100, ledSubsystem));
      SmartDashboard.putData("Led Rainbow", new LedRainbow(ledSubsystem));
    } else {
      shooterVision = new VisionLimeLightSim("limelight-sim", ControlConstants.shooterVisionColumn);
    }
  }

  private void initAutoCommands() {
    command.addOption("Drive Only", new SequentialCommandGroup(
      new SingleCargoPath(), 
      new LowerBall1ToBall2(),
      new LowerBall2ToTerminalA2Path(),
      new TerminalDriveBackA2()
      ));
    if (RobotBase.isReal()) {
      command.addOption("FarM&S", new TarmacTwoBall(transport, new SingleCargoPath()));

      command.addOption("ShortM&S",
          new SequentialCommandGroup(
              new TarmacTwoBall(transport, new LowerTarmacToBall1()),
              new ExtendingShortLeftTurn(transport, new TurnBotLeft())));

      command.addOption("DelayShortM&S",
          new SequentialCommandGroup(
              new Timer(4000),
              new TarmacTwoBall(transport, new LowerTarmacToBall1()),
              new ExtendingShortLeftTurn(transport, new TurnBotLeft())));
      command.addOption("FenderLongM&S",
          new FenderTwoBall(transport, new UpperTarmacToBall()));

      command.addOption("Lower 3 Ball",
          new SequentialCommandGroup(
              new TarmacTwoBall(transport, new LowerTarmacToBall1()),
              new ExtendingThreeBall(transport, new LowerBall1ToBall2())));

      command.addOption("Middle 4 Ball",
          new SequentialCommandGroup(
              new TarmacToBall2(transport, new TarmacToBall2Path()),
              new ExtendingTerminalBall(transport, new LowerBall2ToTerminalA2Path(), new TerminalDriveBackA2())));

      command.addOption("Lower 5 Ball",
          new SequentialCommandGroup(
              new TarmacTwoBall(transport, new LowerTarmacToBall1()),
              new ExtendingThreeBall(transport, new LowerBall1ToBall2()),
              new ExtendingTerminalBall(transport, new LowerBall2ToTerminal(), new TerminalDriveBack())));

      // command.addOption("HubBall2", new AutoMoveAndShoot(transport, shooterVision,
      // new HubToBall2()));

      // command.addOption("ManyBall", new AutoMoveAndShoot(transport, shooterVision,
      // new ManyBallAuto()));
      // command.addOption("Galactic Search", new
      // GalacticSearch(drive.getDriveTrainMain(), shooterVision, drive.getPose()));
    }
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (RobotBase.isReal()) {
      Command ledOrange = new InstantCommand(() -> ledSubsystem.setSolidColor(255, 20, 0));

      Command red = new InstantCommand(() -> shooterVision.setPipeline(1));
      Command blue = new InstantCommand(() -> shooterVision.setPipeline(0));
      Command vTargets = new InstantCommand(() -> shooterVision.setPipeline(2));
      // this adds auto selections in SmartDashboard
      SmartDashboard.putData("red", new SetPipeline(1, shooterVision));
      SmartDashboard.putData("blue", new SetPipeline(0, shooterVision));
      SmartDashboard.putData("default", new SetPipeline(2, shooterVision));

      // toggleLL = new JoystickButton(driver, ControlConstants.toggleLL);

      driver2.createAButton().whenPressed(new InstantCommand(() -> shooterVision.setLight(true), shooterVision));

      // takeSnapshot = new JoystickButton(driver, ControlConstants.takeSnapshot);

      driver2.createBButton().whileHeld(new SnapshotCmd(shooterVision));
    }
    Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay).getLayout("Auto", BuiltInLayouts.kList)
        .withPosition(ControlConstants.autoColumn, 0).withSize(3, 1).add("Choose an Auto Mode", command)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);

  }

  // Just sets up defalt commands (setUpDeftCom)
  public void setUpDeftCom() {
    if (RobotBase.isReal()) {
      // transport.getShooterSubsystem().setHoodCalibrated(false);
      ledSubsystem.setDefaultCommand(new DefaultLed(ledSubsystem, transport));
      cameraSubsystem = new CameraSubsystem();
    }
    if (!DriverStation.isTest()) {
      drive.setUpDeftCom();
      if (RobotBase.isReal())
        transport.setUpDeftCom();
    } else {
      if (RobotBase.isReal())
        transport.setUpDeftCom();
    }

  }

  public void determineAllianceColor() {
    Alliance color = DriverStation.getAlliance();
    if (Alliance.Red.equals(color)) {
      ControlConstants.allianceColor = IndexerConstants.kRedTarget;
      ControlConstants.opposingColor = IndexerConstants.kBlueTarget;
    } else if (Alliance.Blue.equals(color)) {
      ControlConstants.allianceColor = IndexerConstants.kBlueTarget;
      ControlConstants.opposingColor = IndexerConstants.kRedTarget;
    } else {
      ControlConstants.allianceColor = Color.kBlack;
    }
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
