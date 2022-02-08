// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.PistonDeploy;
import frc.robot.commands.PistonRetract;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.SpinIntake;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class Transport {

    private Joystick operator;
    private JoystickButton togglePiston;

    private CANSparkMax intakeMotor;
    private DoubleSolenoid intakePiston;

    private CANSparkMax indexerMotor1;
    private CANSparkMax indexerMotor2;
    private DigitalInput upperBB;
    private IndexerSubsystem indexerSubsystem;

    private IntakeSubsystem intakeSubsystem;
    
    public Transport(Joystick operator){
        
        this.operator = operator;
        togglePiston = new JoystickButton(operator, ControlConstants.intakeUpButton);

        // initializes intake
        intakeMotor = new CANSparkMax(ControlConstants.intakeM, MotorType.kBrushless);
        intakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakePiston);
        intakeSubsystem.setDefaultCommand(new SpinIntake(intakeSubsystem, operator));

        // initializes index
        indexerMotor1 = new CANSparkMax(ControlConstants.lowerIndexM, MotorType.kBrushless);
        indexerMotor2 = new CANSparkMax(ControlConstants.upperIndexM, MotorType.kBrushless);
        upperBB = new DigitalInput(ControlConstants.BB1);
        indexerSubsystem = new IndexerSubsystem(indexerMotor1, upperBB,indexerMotor2);
        indexerSubsystem.setDefaultCommand(new RunIndexer(indexerSubsystem, operator));

        // initalizes turret
        

        SmartDashboard.putData("Piston Retract", new PistonRetract(intakeSubsystem));
        SmartDashboard.putData("Piston Deploy", new PistonDeploy(intakeSubsystem));
    
        togglePiston.whenPressed(new InstantCommand(() -> intakeSubsystem.togglePiston(), intakeSubsystem));
    }
}
