// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.CalibrateDynamicArms;
import frc.robot.commands.CalibrateHood;
import frc.robot.commands.DefaultClimb;
import frc.robot.commands.SpinTurret;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSystem;

/** Add your docs here. */
public class Climb {
    private CANSparkMax leftWinch;
    private CANSparkMax rightWinch;
    
    private DoubleSolenoid climbSolenoid;

    private Joystick driver;
    private Joystick operator;
    private ClimbSubsystem climbSubsystem;

    private JoystickButton climbTime;
    private Trigger climbTogglePistons;
    private ShuffleboardLayout climbEncoderLayout;

    public IntakeSubsystem intakeSubsystem;
    public VisionSystem shooterVision;
    public Transport transport;

    public Climb(Joystick driver, Joystick operator, Transport transport){
        this.driver = driver;
        this.operator = operator;
        this.transport = transport;

        // defined motors
        leftWinch = new CANSparkMax(ControlConstants.leftWinchM,MotorType.kBrushless);
        rightWinch = new CANSparkMax(ControlConstants.rightWinchM,MotorType.kBrushless);


        leftWinch.restoreFactoryDefaults();
        rightWinch.restoreFactoryDefaults();


        // positive pow is counter-clockwise left side
        leftWinch.setInverted(false);
        // positive pow is clockwise right side
        rightWinch.setInverted(true);


        // define solonoid
        climbSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ControlConstants.slot2P, ControlConstants.slot3P);
        climbSolenoid.set(DoubleSolenoid.Value.kReverse);


        // make ClimbSubsystem
        climbSubsystem = new ClimbSubsystem(leftWinch, rightWinch, climbSolenoid);

        // smartdashboard tab
        ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDiagnostics);
        climbEncoderLayout = driverTab.getLayout("Climb Encoders", BuiltInLayouts.kGrid).withPosition(Constants.climbIndex, 0).withSize(3, 5);

        climbEncoderLayout.addBoolean("Is Climb Arms Horizontal", climbSubsystem::isClimbArmHorizontal);
        climbEncoderLayout.addNumber("Left Encoder Value", climbSubsystem::getLeftEncoderValue);
        climbEncoderLayout.addNumber("Right Encoder Value", climbSubsystem::getRightEncoderValue);
        climbEncoderLayout.add("Calibrate Arms Down", new CalibrateDynamicArms(climbSubsystem));

        SmartDashboard.putData("Calibrate Dynamic Arms", new CalibrateDynamicArms(climbSubsystem));

        setBabyCurrentLimits(ControlConstants.neoCurrentLimit, ControlConstants.babyNeoCurrentLimit);
        configureButtonBindings();

    }

    private void setBabyCurrentLimits(int neoCurrentLimit, int babyNeoCurrentLimit){
        leftWinch.setSmartCurrentLimit(80);
        rightWinch.setSmartCurrentLimit(80);
    }

    private void configureButtonBindings(){

        climbTime = new JoystickButton(driver, ControlConstants.climbTime);
    
        climbTime.whileHeld(new SequentialCommandGroup(
                //new CalibrateHood(transport.getShooterSubsystem()),
                new ParallelCommandGroup(
                    new DefaultClimb(climbSubsystem, operator, transport)),
                    new SpinTurret(transport.getTurretSubsystem(), transport.getShooterVision(), false)
                ),
            true);
            //climbTogglePistons = new JoystickButton(operator, ControlConstants.toggleClimb);
            climbTogglePistons = new Trigger(() -> (Math.abs(operator.getRawAxis(ControlConstants.toggleClimb)) > 0));
            climbTogglePistons.whenActive(new InstantCommand(()->climbSubsystem.toggleClimbArm(), climbSubsystem));
        
    
        SmartDashboard.putData("Calibrate Dynamic Arms", new CalibrateDynamicArms(climbSubsystem));
    }
}
