// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.FRC5010.Controller;
import frc.robot.commands.DefaultLed;
import frc.robot.commands.LedBlink;
import frc.robot.commands.LedColor;
import frc.robot.commands.LedRainbow;
import frc.robot.commands.SetPipeline;
import frc.robot.commands.SnapshotCmd;
import frc.robot.commands.Timer;
import frc.robot.commands.auto.ExtendingHangarDefense;
import frc.robot.commands.auto.ExtendingShortLeftTurn;
import frc.robot.commands.auto.ExtendingTerminalBall;
import frc.robot.commands.auto.ExtendingThreeBall;
import frc.robot.commands.auto.FenderTwoBall;
import frc.robot.commands.auto.ShootThenMove;
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
import frc.robot.commands.auto.pathing.UpperBlueBallToUpperRedBall;
import frc.robot.commands.auto.pathing.UpperTarmacToBall;
import frc.robot.commands.auto.pathing.UpperTarmacToBlueBall;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.mechanisms.Climb;
import frc.robot.mechanisms.Drive;
import frc.robot.mechanisms.Transport;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.vision.VisionLimeLightH2;

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

  private Controller driver2 = new Controller(Controller.JoystickPorts.ZERO.ordinal());
  private Controller operator2 = new Controller(Controller.JoystickPorts.ONE.ordinal());

  private SendableChooser<Command> command = new SendableChooser<>();
  private SendableChooser<Command> teamColor = new SendableChooser<>();

  private VisionLimeLightH2 shooterVision;
  private Transport transport;
  private Climb climb;

  private CameraSubsystem cameraSubsystem;
  private LedSubsystem ledSubsystem;

  private Drive drive;

  private JoystickButton toggleLL;

  private JoystickButton takeSnapshot;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driver = new Joystick(ControlConstants.driverJoystick);
    operator = new Joystick(ControlConstants.operatorJoystick);

    // the 20 degrees as of march 15 2022,
    shooterVision = new VisionLimeLightH2("limelight-shooter", 36.5, 37, 102.559, ControlConstants.shooterVisionColumn);
    shooterVision.setPiPMode(0); // removed shooter cam

    drive = new Drive(driver, shooterVision, driver2);
    transport = new Transport(operator, driver, shooterVision);
    climb = new Climb(driver, operator, transport);

    cameraSubsystem = new CameraSubsystem();
    ledSubsystem = new LedSubsystem(0, 62);

    /*
     * command.addOption("LowerCargoToHub", new LowerCargoToHub());
     * command.addOption("HubBall2", new HubToBall2());
     * command.addOption("HubBall3", new HubToBall3());
     * command.addOption("ManyBall", new ManyBallAuto());
     */

    command.addOption("FarM&S", new TarmacTwoBall(transport, new SingleCargoPath()));
    
    command.addOption("ShortM&S", 
      new SequentialCommandGroup(
        new TarmacTwoBall(transport, new LowerTarmacToBall1()),
        new ExtendingShortLeftTurn(transport, new TurnBotLeft())
      )
    );

    command.addOption("FarS&M", 
      new ShootThenMove(transport, new SingleCargoPath())
    );
    
    
    command.addOption("DelayShortM&S", 
      new SequentialCommandGroup(
        new Timer(4000),
        new TarmacTwoBall(transport, new LowerTarmacToBall1()),
        new ExtendingShortLeftTurn(transport, new TurnBotLeft())
      )
    );
    command.addOption("FenderLongM&S", 
      new FenderTwoBall(transport, new UpperTarmacToBall())
      );

    command.addOption("Lower 3 Ball",
      new SequentialCommandGroup(
        new TarmacTwoBall(transport, new LowerTarmacToBall1()),
        new ExtendingThreeBall(transport, new LowerBall1ToBall2())
      )
    );

    command.addOption("Middle 4 Ball", 
      new SequentialCommandGroup(
        new TarmacToBall2(transport, new TarmacToBall2Path()),
        new ExtendingTerminalBall(transport, new LowerBall2ToTerminalA2Path(), new TerminalDriveBackA2())
      )
    );

    command.addOption("Lower 5 Ball",
      new SequentialCommandGroup(
        new TarmacTwoBall(transport, new LowerTarmacToBall1()),
        new ExtendingThreeBall(transport, new LowerBall1ToBall2()),
        new ExtendingTerminalBall(transport, new LowerBall2ToTerminal(), new TerminalDriveBack())
      )
    );

    command.addOption("Upper 2 Ball + Defense",
      new SequentialCommandGroup(
        new TarmacTwoBall(transport, new UpperTarmacToBlueBall()),
        new ExtendingHangarDefense(transport, new UpperBlueBallToUpperRedBall()))
    );
    

    // command.addOption("HubBall2", new AutoMoveAndShoot(transport, shooterVision,
    // new HubToBall2()));
    
    // command.addOption("ManyBall", new AutoMoveAndShoot(transport, shooterVision,
    // new ManyBallAuto()));
    // command.addOption("Galactic Search", new
    // GalacticSearch(drive.getDriveTrainMain(), shooterVision, drive.getPose()));

    teamColor.setDefaultOption("VTargets", new InstantCommand(() -> shooterVision.setPipeline(2)));
    teamColor.addOption("Red", new InstantCommand(() -> shooterVision.setPipeline(1)));
    teamColor.addOption("Blue", new InstantCommand(() -> shooterVision.setPipeline(0)));

    // Configure the button bindings
    configureButtonBindings();
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
    Command red = new InstantCommand(() -> shooterVision.setPipeline(1));
    Command blue = new InstantCommand(() -> shooterVision.setPipeline(0));
    Command vTargets = new InstantCommand(() -> shooterVision.setPipeline(2));
    Command ledOrange = new InstantCommand(() -> ledSubsystem.setSolidColor(255, 20, 0));
    // this adds auto selections in SmartDashboard
    Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay).getLayout("Auto", BuiltInLayouts.kList)
        .withPosition(ControlConstants.autoColumn, 0).withSize(3, 1).add("Choose an Auto Mode", command)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);

    SmartDashboard.putData("red", new SetPipeline(1, shooterVision));
    SmartDashboard.putData("blue", new SetPipeline(0, shooterVision));
    SmartDashboard.putData("default", new SetPipeline(2, shooterVision));
    SmartDashboard.putData("Leds Orange", new LedColor(255, 25, 0, ledSubsystem));
    SmartDashboard.putData("Leds Green", new LedColor(0, 255, 0, ledSubsystem));
    SmartDashboard.putData("Leds Off", new LedColor(0, 0, 0, ledSubsystem));
    SmartDashboard.putData("Led Blink Blue", new LedBlink(0, 0, 255, 100, ledSubsystem));
    SmartDashboard.putData("Led Rainbow", new LedRainbow(ledSubsystem));

    // toggleLL = new JoystickButton(driver, ControlConstants.toggleLL);

    driver2.createAButton().whenPressed(new InstantCommand(() -> shooterVision.toggleLight(), shooterVision));

    // takeSnapshot = new JoystickButton(driver, ControlConstants.takeSnapshot);

    driver2.createBButton().whileHeld(new SnapshotCmd(shooterVision));

  }

  // Just sets up defalt commands (setUpDeftCom)
  public void setUpDeftCom() {
    //transport.getShooterSubsystem().setHoodCalibrated(false);
    ledSubsystem.setDefaultCommand(new DefaultLed(ledSubsystem, transport));
    if (!DriverStation.isTest()) {
      drive.setUpDeftCom();
      transport.setUpDeftCom();
    } else {
      transport.setUpDeftCom();
    }
    cameraSubsystem = new CameraSubsystem();
  }

  public void determineAllianceColor(){
    Alliance color = DriverStation.getAlliance();
    if (Alliance.Red.equals(color)){
      ControlConstants.allianceColor = IndexerConstants.kRedTarget;
      ControlConstants.opposingColor = IndexerConstants.kBlueTarget;
    } else if (Alliance.Blue.equals(color)){
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
