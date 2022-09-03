// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.FRC5010.GenericEncoder;
import frc.robot.FRC5010.Impl.SimulatedEncoder;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IndexerConstants;

public class IntakeSubsystem extends SubsystemBase {
  DoubleSolenoid intakePiston;
  ColorSensorV3 colorSensor;
  ColorMatch m_colorMatcher;
  CANSparkMax intakeMotor;
  ShuffleboardLayout intakeLayout;
  FlywheelSim intakeMotorSim;
  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor intakeGearBox = DCMotor.getNEO(1);
  private Encoder intakeEncoder;
  private GenericEncoder intakeSimEncoder;
  private EncoderSim intakeEncoderSimCalc;
  private final MechanismRoot2d intakePivot = RobotContainer.mech2d.getRoot("IntakePivot", 10, 7);
  private final MechanismRoot2d intakeMotor2d = RobotContainer.mech2d.getRoot("IntakeMotor", 3, 17);
  private final MechanismLigament2d intakeArm = intakePivot.append(
      new MechanismLigament2d(
          "IntakeArm",
          10,
          90,
          6,
          new Color8Bit(Color.kYellow)));
  private final MechanismLigament2d intakeMotorDial = intakeMotor2d.append(
      new MechanismLigament2d(
          "IntakeMotorDial",
          3,
          0,
          6,
          new Color8Bit(Color.kBlue)));

  private boolean isRejectOn = true;

  public IntakeSubsystem(CANSparkMax intakeMotor, DoubleSolenoid intakePiston, ColorSensorV3 colorSensor) {
    this.intakeMotor = intakeMotor;
    this.colorSensor = colorSensor;
    this.intakePiston = intakePiston;

    this.m_colorMatcher = new ColorMatch();
    m_colorMatcher.addColorMatch(IndexerConstants.kRedTarget);
    m_colorMatcher.addColorMatch(IndexerConstants.kBlueTarget);
    m_colorMatcher.addColorMatch(Color.kBlack);
    m_colorMatcher.addColorMatch(Color.kWhite);
    m_colorMatcher.addColorMatch(Color.kGreen);
    m_colorMatcher.addColorMatch(Color.kOrange);
    m_colorMatcher.addColorMatch(Color.kYellow);

    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay);
    intakeLayout = driverTab.getLayout("Shooter", BuiltInLayouts.kGrid).withPosition(Constants.indexerIndex, 0)
        .withSize(1, 5);
    intakeLayout.addString("Color", this::getColorString);
    intakeLayout.addNumber("Confidence", this::getConfidence);
    intakeLayout.addBoolean("Reject Ball", this::getRejectState);

    if (Robot.isSimulation()) {
      intakeMotorSim = new FlywheelSim(intakeGearBox, 1, 1);
      intakeEncoder = new Encoder(20, 21);
      intakeSimEncoder = new SimulatedEncoder(intakeEncoder);
      intakeEncoderSimCalc = new EncoderSim(intakeEncoder);
    }
  }

  public Color getColor() {
    return m_colorMatcher.matchClosestColor(colorSensor.getColor()).color;
  }

  public double getConfidence() {
    return m_colorMatcher.matchClosestColor(colorSensor.getColor()).confidence;
  }

  public String getColorString() {
    Color color = m_colorMatcher.matchClosestColor(colorSensor.getColor()).color;
    if (color.equals(IndexerConstants.kBlueTarget)) {
      return "blue";
    } else if (color.equals(IndexerConstants.kRedTarget)) {
      return "red";
    } else {
      return "unknown";
    }
  }

  public void toggleReject() {
    isRejectOn = !isRejectOn;
  }

  public boolean getRejectState() {
    return isRejectOn;
  }

  public void togglePiston() {
    intakePiston.toggle();
  }

  public void pistonOff() {
    intakePiston.set(DoubleSolenoid.Value.kOff);
  }

  public void retractIntake() {
    intakePiston.set(DoubleSolenoid.Value.kReverse);
  }

  public void deployIntake() {
    intakePiston.set(DoubleSolenoid.Value.kForward);
  }

  public boolean isIntakeDeployed() {
    return intakePiston.get().equals(DoubleSolenoid.Value.kForward);
  }

  public void setIntakePow(double pow) {
    intakeMotor.set(pow);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    intakeMotorSim.setInput(intakeMotor.get() * RobotController.getBatteryVoltage());
    // Next, we update it. The standard loop time is 20ms.
    intakeMotorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    intakeSimEncoder.setPosition(intakeEncoderSimCalc.getDistance());
    intakeSimEncoder.setRate(intakeEncoderSimCalc.getRate());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(intakeMotorSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    intakeArm.setAngle(intakePiston.get() == DoubleSolenoid.Value.kForward ? 135 : 90);
    intakeMotorDial.setAngle(180 * intakeMotor.get());
  }
}
