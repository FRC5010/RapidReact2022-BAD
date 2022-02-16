// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultShoot;
import frc.robot.commands.Launcher;
import frc.robot.commands.MoveHood;
import frc.robot.commands.PistonDeploy;
import frc.robot.commands.PistonRetract;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinTurret;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSystem;

/** Add your docs here. */
public class Transport {

    private Joystick operator;
    private JoystickButton togglePiston;

    private VisionSystem shooterVision;

    private CANSparkMax intakeMotor;
    private DoubleSolenoid intakePiston;
    private IntakeSubsystem intakeSubsystem;

    private CANSparkMax lowerIndexMotor;
    private CANSparkMax upperIndexerMotor;
    private DigitalInput upperBB;
    private IndexerSubsystem indexerSubsystem;

    private CANSparkMax turretMotor;
    private TurretSubsystem turretSubsystem;

    private CANSparkMax flyWheelLeft;
    private CANSparkMax flyWheelRight;
    private CANSparkMax hoodMotor;
    private CANSparkMax feederMotor;
    private ShooterSubsystem shooterSubsystem;

    private JoystickButton hoodDown;
    private JoystickButton hoodUp;
    private JoystickButton manualShoot; 
    
    public Transport(Joystick operator, VisionSystem shooterVision){
        this.shooterVision = shooterVision;
        
        this.operator = operator;
        togglePiston = new JoystickButton(operator, ControlConstants.intakeUpButton);

        // initializes intake
        intakeMotor = new CANSparkMax(ControlConstants.intakeM, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
        intakePiston.set(Value.kReverse);

        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakePiston);
        

        // initializes index
        lowerIndexMotor = new CANSparkMax(ControlConstants.lowerIndexM, MotorType.kBrushless);
        lowerIndexMotor.restoreFactoryDefaults();
        upperIndexerMotor = new CANSparkMax(ControlConstants.upperIndexM, MotorType.kBrushless);


        upperIndexerMotor.restoreFactoryDefaults();

        // Postive voltage is in and up
        lowerIndexMotor.setInverted(false);
        upperIndexerMotor.setInverted(false);

        upperBB = new DigitalInput(ControlConstants.BB1);

        indexerSubsystem = new IndexerSubsystem(lowerIndexMotor, upperIndexerMotor, upperBB);
        
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
        feederMotor = new CANSparkMax(ControlConstants.feederWheelM, MotorType.kBrushless);

        // postive voltage is ball up 
        feederMotor.restoreFactoryDefaults();
        
        // Positve voltage spins fly wheel forward 
        flyWheelRight.setInverted(true);
        flyWheelLeft.follow(flyWheelRight, true);
        flyWheelRight.setOpenLoopRampRate(0.5);
        shooterSubsystem = new ShooterSubsystem(flyWheelRight, hoodMotor, feederMotor, shooterVision);
        

        SmartDashboard.putData("Piston Retract", new PistonRetract(intakeSubsystem));
        SmartDashboard.putData("Piston Deploy", new PistonDeploy(intakeSubsystem));
    
        togglePiston.whenPressed(new InstantCommand(() -> intakeSubsystem.togglePiston(), intakeSubsystem));
        setBabyCurrentLimits(ControlConstants.neoCurrentLimit, ControlConstants.babyNeoCurrentLimit);
        configureButtonBindings();
    }

    private void setBabyCurrentLimits(int neoCurrentLimit, int babyNeoCurrentLimit) {
        intakeMotor.setSmartCurrentLimit(neoCurrentLimit);
        lowerIndexMotor.setSmartCurrentLimit(babyNeoCurrentLimit);
        upperIndexerMotor.setSmartCurrentLimit(babyNeoCurrentLimit);
        turretMotor.setSmartCurrentLimit(babyNeoCurrentLimit);
        flyWheelLeft.setSmartCurrentLimit(neoCurrentLimit);
        flyWheelRight.setSmartCurrentLimit(neoCurrentLimit);
        hoodMotor.setSmartCurrentLimit(babyNeoCurrentLimit);
        feederMotor.setSmartCurrentLimit(neoCurrentLimit);
    }

    public void configureButtonBindings(){
        hoodDown = new JoystickButton(operator, ControlConstants.hoodDown);
        hoodUp = new JoystickButton(operator, ControlConstants.hoodUp);

        hoodDown.whenPressed(new MoveHood(shooterSubsystem, -10));
        hoodUp.whenPressed(new MoveHood(shooterSubsystem, 10));

        manualShoot = new JoystickButton(operator, ControlConstants.launchButton);
        manualShoot.whileHeld(new DefaultShoot(shooterSubsystem, indexerSubsystem), true);
    }
    public void setUpDeftCom(){
        shooterSubsystem.setDefaultCommand(new Launcher(shooterSubsystem, operator));
        indexerSubsystem.setDefaultCommand(new RunIndexer(indexerSubsystem, operator));
        intakeSubsystem.setDefaultCommand(new SpinIntake(intakeSubsystem, indexerSubsystem, operator));
        turretSubsystem.setDefaultCommand(new SpinTurret(turretSubsystem, operator));

    }
}
