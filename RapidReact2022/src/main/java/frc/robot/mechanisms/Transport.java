// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.FRC5010.Controller;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.CalibrateHood;
import frc.robot.commands.DefaultShoot;
import frc.robot.commands.FenderShot;
import frc.robot.commands.LockAndLoad;
import frc.robot.commands.MoveHood;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.SnapshotCmd;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinTurret;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.DiagonalIndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;
import frc.robot.subsystems.vision.VisionSystem;

/** Add your docs here. */
public class Transport {

    private Joystick operator;
    private Joystick driver;
    private Controller operator2;
    private Controller driver2;
    private JoystickButton togglePiston;

    private VisionSystem shooterVision;

    private CANSparkMax intakeMotor;
    private DoubleSolenoid intakePiston;
    private ColorSensorV3 colorSensor;
    private IntakeSubsystem intakeSubsystem;

    private CANSparkMax diagonalLowerMotor;
    private CANSparkMax diagonalUpperMotor;
    private CANSparkMax verticalLongMotor;
    private CANSparkMax verticalShortMotor;
    private DigitalInput lowerBB;
    private DigitalInput upperBB;

    private DiagonalIndexerSubsystem indexerSubsystem;
    private VerticalIndexerSubsystem upperIndexerSubsystem;

    private CANSparkMax turretMotor;
    private TurretSubsystem turretSubsystem;

    private CANSparkMax flyWheelLeft;
    private CANSparkMax flyWheelRight;
    private CANSparkMax hoodMotor;
    private CANSparkMax feederMotor;
    private ShooterSubsystem shooterSubsystem;

    private JoystickButton hoodDown;
    private JoystickButton hoodUp;
    private JoystickButton aimAndShoot;
    private JoystickButton calibrateHood;
    private JoystickButton indexerUp;
    private JoystickButton indexerDown;
    private POVButton incFlyWheel;
    private POVButton decFlyWheel;
    private JoystickButton fenderShot;
    private JoystickButton defaultShoot;
    private JoystickButton driveYEET;
    private JoystickButton fenderShot2;
    private JoystickButton climbTime;
    private Trigger lockAndLoad;
    private Trigger intakeTrigger;
    private JoystickButton toggleReject;
    private Trigger indexerTrigger;
    private Trigger indexerTriggerDown;
    private Trigger indexerTriggerUp;
    private POVButton shotAdjustmentDec;
    private POVButton shotAdjustmentInc;

    public Transport(Joystick operator, Joystick driver, VisionSystem shooterVision, Controller operator2,
            Controller driver2) {
        this.shooterVision = shooterVision;
        this.driver = driver;
        this.operator = operator;
        this.driver2 = driver2;
        this.operator2 = operator2;
        // togglePiston = new JoystickButton(driver, ControlConstants.toggleIntake);

        // initializes intake
        intakeMotor = new CANSparkMax(ControlConstants.intakeM, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setOpenLoopRampRate(0.5);
        intakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, ControlConstants.slot0P, ControlConstants.slot1P);
        intakePiston.set(Value.kReverse);
        colorSensor = new ColorSensorV3(ControlConstants.i2cPort);
        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakePiston, colorSensor);

        // initializes index
        diagonalLowerMotor = new CANSparkMax(ControlConstants.diagonalLowerM, MotorType.kBrushless);
        diagonalLowerMotor.restoreFactoryDefaults();
        verticalLongMotor = new CANSparkMax(ControlConstants.verticalLongM, MotorType.kBrushless);
        verticalLongMotor.restoreFactoryDefaults();
        diagonalUpperMotor = new CANSparkMax(ControlConstants.diagonalUpperM, MotorType.kBrushless);
        diagonalUpperMotor.restoreFactoryDefaults();
        verticalShortMotor = new CANSparkMax(ControlConstants.verticalShortM, MotorType.kBrushless);
        verticalShortMotor.restoreFactoryDefaults();
        // Postive voltage is in and up
        diagonalLowerMotor.setInverted(false);
        verticalLongMotor.setInverted(true);
        diagonalUpperMotor.setInverted(true);
        verticalShortMotor.setInverted(false);

        lowerBB = new DigitalInput(ControlConstants.BB1);
        upperBB = new DigitalInput(ControlConstants.BB2);

        indexerSubsystem = new DiagonalIndexerSubsystem(diagonalLowerMotor, diagonalUpperMotor, lowerBB);
        upperIndexerSubsystem = new VerticalIndexerSubsystem(verticalShortMotor, verticalLongMotor, upperBB);

        // initializes turret
        turretMotor = new CANSparkMax(ControlConstants.turretM, MotorType.kBrushless);

        // Postive voltage moves clockwise
        turretMotor.restoreFactoryDefaults();
        turretMotor.setInverted(true);

        turretSubsystem = new TurretSubsystem(turretMotor, shooterVision);

        // initialize shooter
        flyWheelLeft = new CANSparkMax(ControlConstants.leftFlyWheelM, MotorType.kBrushless);
        flyWheelLeft.restoreFactoryDefaults();
        flyWheelRight = new CANSparkMax(ControlConstants.rightFlyWheelM, MotorType.kBrushless);
        flyWheelRight.restoreFactoryDefaults();

        hoodMotor = new CANSparkMax(ControlConstants.hoodM, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();

        // positive voltage moves hood up
        hoodMotor.setInverted(true);
        feederMotor = new CANSparkMax(ControlConstants.feederWheelM, MotorType.kBrushless);

        // postive voltage is ball up
        feederMotor.restoreFactoryDefaults();

        // Positve voltage spins fly wheel forward
        flyWheelRight.setInverted(true);
        flyWheelLeft.follow(flyWheelRight, true);
        flyWheelRight.setOpenLoopRampRate(0.5);
        shooterSubsystem = new ShooterSubsystem(flyWheelRight, hoodMotor, feederMotor, shooterVision);

        // climbTime.whileHeld(new InstantCommand(() -> intakeSubsystem.deployIntake(),
        // intakeSubsystem));
        setBabyCurrentLimits(ControlConstants.neoCurrentLimit, ControlConstants.babyNeoCurrentLimit);

        configureButtonBindings();
    }

    private void setBabyCurrentLimits(int neoCurrentLimit, int babyNeoCurrentLimit) {
        intakeMotor.setSmartCurrentLimit(60);
        diagonalLowerMotor.setSmartCurrentLimit(babyNeoCurrentLimit);
        verticalLongMotor.setSmartCurrentLimit(babyNeoCurrentLimit);
        turretMotor.setSmartCurrentLimit(50);
        flyWheelLeft.setSmartCurrentLimit(neoCurrentLimit);
        flyWheelRight.setSmartCurrentLimit(neoCurrentLimit);
        hoodMotor.setSmartCurrentLimit(babyNeoCurrentLimit);
        feederMotor.setSmartCurrentLimit(60);
        diagonalUpperMotor.setSmartCurrentLimit(babyNeoCurrentLimit);
    }

    public void configureButtonBindings() {
        intakeTrigger = new Trigger(() -> (Math.abs(
                driver.getRawAxis(ControlConstants.intakeAxis)
                        - driver.getRawAxis(ControlConstants.outtakeAxis)) > 0));
        // intakeTrigger.whileActiveOnce(new SequentialCommandGroup(new
        // SpinIntake(intakeSubsystem, indexerSubsystem, driver, 200), ));
        intakeTrigger
                .whileActiveOnce(
                        new SpinIntake(intakeSubsystem, indexerSubsystem, upperIndexerSubsystem, driver, 150));

        // hoodDown = new JoystickButton(operator, ControlConstants.hoodDown); // B
        // Button
        // hoodUp = new JoystickButton(operator, ControlConstants.hoodUp); // Y Button

        operator2.createBButton()
                .whenPressed(new MoveHood(shooterSubsystem, -ShooterConstants.HoodConstants.hoodTolerance));
        operator2.createYButton()
                .whenPressed(new MoveHood(shooterSubsystem, ShooterConstants.HoodConstants.hoodTolerance));

        // aimAndShoot = new JoystickButton(operator, ControlConstants.launchButton); //
        // right bumper
        // switched out DefaultShoot for AimAndShoot
        operator2.createRightBumper().whileHeld(
                new ParallelDeadlineGroup(
                        new AimAndShoot(shooterSubsystem, upperIndexerSubsystem, indexerSubsystem, shooterVision),
                        new SnapshotCmd(shooterVision)),
                false);

        // defaultShoot = new JoystickButton(operator, ControlConstants.defaultShoot);
        // // Start Button
        operator2.createStartButton()
                .whileHeld(new DefaultShoot(shooterSubsystem, upperIndexerSubsystem, indexerSubsystem));

        // incFlyWheel = new POVButton(operator, ControlConstants.incShooter); //
        // POVButton UP

        operator2.createUpPovButton().whenPressed(
                new InstantCommand(() -> ShooterConstants.defaultFlyWheelRPM += ShooterConstants.changeSetPoint));

        // decFlyWheel = new POVButton(operator, ControlConstants.decShooter); //
        // POVButton DOWN
        operator2.createDownPovButton().whenPressed(
                new InstantCommand(() -> ShooterConstants.defaultFlyWheelRPM -= ShooterConstants.changeSetPoint));

        // calibrateHood = new JoystickButton(driver, ControlConstants.calibrate); //
        // Driver START_BUTTON
        driver2.createStartButton().whenPressed(new CalibrateHood(shooterSubsystem));

        // indexerUp = new JoystickButton(operator, ControlConstants.indexerUp);
        // indexerUp.whileHeld(new RunIndexer(upperIndexerSubsystem, indexerSubsystem,
        // IndexerConstants.indexerRPM), false);

        // indexerDown = new JoystickButton(operator, ControlConstants.indexerDown);
        // indexerDown.whileHeld(new RunIndexer(upperIndexerSubsystem, indexerSubsystem,
        // -IndexerConstants.indexerRPM), false);

        indexerTrigger = new Trigger(() -> Math.abs(operator.getRawAxis(ControlConstants.joystickIndexer)) > .08);
        indexerTrigger.whileActiveOnce(new RunIndexer(upperIndexerSubsystem, indexerSubsystem, operator));

        // fenderShot = new JoystickButton(operator, ControlConstants.fenderButton);
        // Operator Left Bumper
        operator2.createLeftBumper()
                .whileHeld(new FenderShot(shooterSubsystem, upperIndexerSubsystem, indexerSubsystem, true), false);

        // fenderShot2 = new JoystickButton(operator, ControlConstants.fender2Button);
        // Operator A Button
        operator2.createAButton()
                .whileHeld(new FenderShot(shooterSubsystem, upperIndexerSubsystem, indexerSubsystem, false), false);

        // lockAndLoad = new JoystickButton(operator,
        // ControlConstants.lockAndLoadButton);
        // lockAndLoad.whenPressed(new LockAndLoad(upperIndexerSubsystem,
        // indexerSubsystem, shooterSubsystem, shooterVision), true);
        lockAndLoad = new Trigger(() -> (Math.abs(operator.getRawAxis(ControlConstants.lockAndLoadButton)) > 0));
        lockAndLoad.whenActive(
                new LockAndLoad(upperIndexerSubsystem, indexerSubsystem, shooterSubsystem, shooterVision), true);

        // toggleReject = new JoystickButton(operator, ControlConstants.toggleReject);
        // // Operator X_BUTTON
        operator2.createXButton()
                .whenPressed(new InstantCommand(() -> intakeSubsystem.toggleReject(), intakeSubsystem));
        // finish way to toggle the boolean in intakesubsystem

        // shotAdjustmentInc = new POVButton(operator,
        // ControlConstants.shotAdjustmentUp); Operator POVButton Right
        operator2.createRightPovButton()
                .whenPressed(new InstantCommand(() -> ShooterConstants.shotAdjustment += 25));

        // shotAdjustmentDec = new POVButton(operator,
        // ControlConstants.shotAdjustmentDown); Operator POVButton Left
        operator2.createLeftPovButton()
                .whenPressed(new InstantCommand(() -> ShooterConstants.shotAdjustment -= 25));

    }

    public void setUpDeftCom() {
        // shooterSubsystem.setDefaultCommand(new Launcher(shooterSubsystem, operator));
        // intakeSubsystem.setDefaultCommand(new SpinIntake(intakeSubsystem,
        // indexerSubsystem, driver));
        if (Robot.isReal()) {
            turretSubsystem.setDefaultCommand(new SpinTurret(turretSubsystem, shooterVision, operator, false));
        }
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public TurretSubsystem getTurretSubsystem() {
        return turretSubsystem;
    }

    public DiagonalIndexerSubsystem getDiagonalIndexerSubsystem() {
        return indexerSubsystem;
    }

    public VerticalIndexerSubsystem getVerticalIndexerSubsystem() {
        return upperIndexerSubsystem;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    public VisionSystem getShooterVision() {
        return shooterVision;
    }
}
