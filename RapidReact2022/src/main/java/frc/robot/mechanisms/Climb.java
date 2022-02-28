// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultClimb;
import frc.robot.commands.DefaultShoot;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class Climb {
    private CANSparkMax leftWinch;
    private CANSparkMax rightWinch;
    private CANSparkMax staticHooks;

    private DoubleSolenoid climbSolenoid;

    private Joystick driver;
    private Joystick operator;
    private ClimbSubsystem climbSubsystem;

    private ShooterSubsystem shooterSubsystem;

    private JoystickButton climbTime;
    private JoystickButton climbToggle;


    public Climb(Joystick driver, Joystick operator){
        this.driver = driver;
        this.operator = operator;

        // defined motors
        leftWinch = new CANSparkMax(ControlConstants.LeftWinchM,MotorType.kBrushless);
        rightWinch = new CANSparkMax(ControlConstants.RightWinchM,MotorType.kBrushless);
        staticHooks = new CANSparkMax(ControlConstants.StaticHooksM,MotorType.kBrushless);

        leftWinch.restoreFactoryDefaults();
        rightWinch.restoreFactoryDefaults();
        staticHooks.restoreFactoryDefaults();

        // positive pow is up
        leftWinch.setInverted(false);
        rightWinch.setInverted(false);
        staticHooks.setInverted(false);

        // define solonoid
        climbSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ControlConstants.slot2P, ControlConstants.slot3P);


        // make ClimbSubsystem
        climbSubsystem = new ClimbSubsystem(leftWinch, rightWinch, staticHooks, climbSolenoid);

        setBabyCurrentLimits(ControlConstants.neoCurrentLimit, ControlConstants.babyNeoCurrentLimit);
        configureButtonBindings();

    }

    private void setBabyCurrentLimits(int neoCurrentLimit, int babyNeoCurrentLimit){
        leftWinch.setSmartCurrentLimit(neoCurrentLimit);
        rightWinch.setSmartCurrentLimit(neoCurrentLimit);
        staticHooks.setSmartCurrentLimit(neoCurrentLimit);
    }

    private void configureButtonBindings(){
        climbTime = new JoystickButton(operator, ControlConstants.climbTime);
        climbTime.whileHeld(new DefaultClimb(climbSubsystem, driver), true);
        climbToggle = new JoystickButton(driver, ControlConstants.toggleClimb);
        climbToggle.whenPressed(new InstantCommand(()->climbSubsystem.toggleClimbArm(), climbSubsystem, shooterSubsystem));

        


    }
}
