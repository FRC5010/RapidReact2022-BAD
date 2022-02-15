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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Launcher;
import frc.robot.commands.MoveHood;
import frc.robot.commands.PistonDeploy;
import frc.robot.commands.PistonRetract;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinShooter;
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

    private CANSparkMax indexerMotor1;
    private CANSparkMax indexerMotor2;
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
    
    public Transport(Joystick operator, VisionSystem shooterVision){
        this.shooterVision = shooterVision;
        
        this.operator = operator;
        togglePiston = new JoystickButton(operator, ControlConstants.intakeUpButton);

        // initializes intake
        intakeMotor = new CANSparkMax(ControlConstants.intakeM, MotorType.kBrushless);
        intakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakePiston);
        

        // initializes index
        indexerMotor1 = new CANSparkMax(ControlConstants.lowerIndexM, MotorType.kBrushless);
        indexerMotor2 = new CANSparkMax(ControlConstants.upperIndexM, MotorType.kBrushless);
        upperBB = new DigitalInput(ControlConstants.BB1);

        indexerSubsystem = new IndexerSubsystem(indexerMotor1, indexerMotor2, upperBB);
        
        // initializes turret
        turretMotor = new CANSparkMax(ControlConstants.turretM, MotorType.kBrushless);

        turretSubsystem = new TurretSubsystem(turretMotor, shooterVision);




        // initialize shooter
        flyWheelLeft = new CANSparkMax(ControlConstants.leftFlyWheelM, MotorType.kBrushless);
        flyWheelRight = new CANSparkMax(ControlConstants.rightFlyWheelM, MotorType.kBrushless);
        hoodMotor = new CANSparkMax(ControlConstants.hoodM, MotorType.kBrushless);
        feederMotor = new CANSparkMax(ControlConstants.feederWheelM, MotorType.kBrushless);

        flyWheelLeft.follow(flyWheelRight, true);
        flyWheelRight.setOpenLoopRampRate(0.5);
        shooterSubsystem = new ShooterSubsystem(flyWheelRight, hoodMotor, feederMotor, shooterVision);
        

        SmartDashboard.putData("Piston Retract", new PistonRetract(intakeSubsystem));
        SmartDashboard.putData("Piston Deploy", new PistonDeploy(intakeSubsystem));
        SmartDashboard.putData("Shoot", new SpinShooter(shooterSubsystem, -.6, .8));
        SmartDashboard.putData("Stop Shooter", new SpinShooter(shooterSubsystem, 0, 0));
    
        togglePiston.whenPressed(new InstantCommand(() -> intakeSubsystem.togglePiston(), intakeSubsystem));
        setBabyCurrentLimits(ControlConstants.neoCurrentLimit, ControlConstants.babyNeoCurrentLimit);
        configureButtonBindings();
    }

    private void setBabyCurrentLimits(int neoCurrentLimit, int babyNeoCurrentLimit) {
        intakeMotor.setSmartCurrentLimit(neoCurrentLimit);
        indexerMotor1.setSmartCurrentLimit(babyNeoCurrentLimit);
        indexerMotor2.setSmartCurrentLimit(babyNeoCurrentLimit);
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
    }
    public void setUpDeftCom(){
        shooterSubsystem.setDefaultCommand(new Launcher(shooterSubsystem, operator));
        indexerSubsystem.setDefaultCommand(new RunIndexer(indexerSubsystem, operator));
        intakeSubsystem.setDefaultCommand(new SpinIntake(intakeSubsystem, operator));

    }
}
