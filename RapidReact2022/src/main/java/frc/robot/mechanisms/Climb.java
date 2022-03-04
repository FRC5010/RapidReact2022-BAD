// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CalibrateDynamicArms;
import frc.robot.commands.DefaultClimb;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.ClimbSubsystem;

/** Add your docs here. */
public class Climb {
    private CANSparkMax leftWinch;
    private CANSparkMax rightWinch;
    private CANSparkMax staticHooks;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private RelativeEncoder staticEncoder;

    private DoubleSolenoid climbSolenoid;

    private Joystick driver;
    private Joystick operator;
    private ClimbSubsystem climbSubsystem;

    private JoystickButton climbTime;
    private JoystickButton climbToggle;
    private ShuffleboardLayout climbEncoderLayout;

    public Climb(Joystick driver, Joystick operator){
        this.driver = driver;
        this.operator = operator;

        // defined motors
        leftWinch = new CANSparkMax(ControlConstants.leftWinchM,MotorType.kBrushless);
        rightWinch = new CANSparkMax(ControlConstants.rightWinchM,MotorType.kBrushless);
        staticHooks = new CANSparkMax(ControlConstants.staticHooksM,MotorType.kBrushless);


        leftWinch.restoreFactoryDefaults();
        rightWinch.restoreFactoryDefaults();
        staticHooks.restoreFactoryDefaults();

        // positive pow is counter-clockwise left side
        leftWinch.setInverted(false);
        // positive pow is clockwise right side
        rightWinch.setInverted(true);
        // positive pow is clockwise right side
        staticHooks.setInverted(false);

        // define solonoid
        climbSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ControlConstants.slot2P, ControlConstants.slot3P);
        climbSolenoid.set(DoubleSolenoid.Value.kReverse);


        // make ClimbSubsystem
        climbSubsystem = new ClimbSubsystem(leftWinch, rightWinch, staticHooks, climbSolenoid);

        // smartdashboard tab
        int colIndex = 0;
        ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabClimbDisplay);
        climbEncoderLayout = driverTab.getLayout("Climb Encoders", BuiltInLayouts.kGrid).withPosition(colIndex, 0).withSize(3, 5);

        climbEncoderLayout.addBoolean("Is Climb Arms Horizontal", climbSubsystem::isClimbArmHorizontal);
        climbEncoderLayout.addNumber("Left Encoder Value", climbSubsystem::getLeftEncoderValue);
        climbEncoderLayout.addNumber("Right Encoder Value", climbSubsystem::getRightEncoderValue);
        climbEncoderLayout.addNumber("Static Encoder Value", climbSubsystem::getStaticEncoderValue);
        climbEncoderLayout.add("Calibrate Arms Down", new CalibrateDynamicArms(climbSubsystem));

        SmartDashboard.putData("Calibrate Dynamic Arms", new CalibrateDynamicArms(climbSubsystem));

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
        climbToggle.whenPressed(new InstantCommand(()->climbSubsystem.toggleClimbArm(), climbSubsystem));
    
        SmartDashboard.putData("Calibrate Dynamic Arms", new CalibrateDynamicArms(climbSubsystem));
    }
}
