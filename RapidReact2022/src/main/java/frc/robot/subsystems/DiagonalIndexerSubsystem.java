// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.jsontype.impl.LaissezFaireSubTypeValidator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.IndexerConstants;

public class DiagonalIndexerSubsystem extends SubsystemBase {
  /** Creates a new Indexer. */
  private CANSparkMax diagonalLowerMotor;
  private CANSparkMax diagonalUpperMotor;
  private DigitalInput lowerBB;

  private SparkMaxPIDController lowerPIDController;
  private SparkMaxPIDController upperPIDController;

  private RelativeEncoder upperEncoder;
  private RelativeEncoder lowerEncoder;

  private ShuffleboardLayout indexerLayout;
  private double indexerSetPoint;

  FlywheelSim lowerDiagMotorSim;
  FlywheelSim upperDiagMotorSim;
  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor lowerDiagIndexerGearBox = DCMotor.getNEO(1);
  private final DCMotor upperDiagIndexerGearBox = DCMotor.getNEO(1);

  private final MechanismRoot2d diagLowerIndexer2d = RobotContainer.mech2d.getRoot("DiagLowerIndexer", 20, 10);
  private final MechanismRoot2d diagUpperIndexer2d = RobotContainer.mech2d.getRoot("DiagUpperIndexer", 20, 20);
  private final MechanismLigament2d diagLowerIndexerDial = diagLowerIndexer2d.append(
      new MechanismLigament2d(
          "DiagLowerIndexerDial",
          3,
          0,
          6,
          new Color8Bit(Color.kBlue)));
  private final MechanismLigament2d diagUpperIndexerDial = diagUpperIndexer2d.append(
      new MechanismLigament2d(
          "DiagUpperIndexerDial",
          3,
          0,
          6,
          new Color8Bit(Color.kBlue)));
  private final MechanismLigament2d diagLowerIndexer = diagLowerIndexer2d.append(
      new MechanismLigament2d(
          "DiagLowerIndexer",
          10,
          200,
          6,
          new Color8Bit(Color.kOrange)));
  private final MechanismLigament2d diagUpperIndexer = diagUpperIndexer2d.append(
      new MechanismLigament2d(
          "DiagUpperIndexer",
          10,
          200,
          6,
          new Color8Bit(Color.kOrange)));

  public DiagonalIndexerSubsystem(CANSparkMax diagonalLowerMotor, CANSparkMax diagonalUpperMotor,
      DigitalInput upperBB) {
    this.lowerBB = upperBB;
    this.diagonalLowerMotor = diagonalLowerMotor;
    this.diagonalUpperMotor = diagonalUpperMotor;
    this.lowerPIDController = diagonalLowerMotor.getPIDController();
    lowerPIDController.setP(IndexerConstants.DiagonalLower.kP);
    lowerPIDController.setI(IndexerConstants.DiagonalLower.kI);
    lowerPIDController.setD(IndexerConstants.DiagonalLower.kD);

    this.upperPIDController = diagonalUpperMotor.getPIDController();
    upperPIDController.setP(IndexerConstants.DiagonalUpper.kP);
    upperPIDController.setI(IndexerConstants.DiagonalUpper.kI);
    upperPIDController.setD(IndexerConstants.DiagonalUpper.kD);

    upperEncoder = diagonalUpperMotor.getEncoder();
    lowerEncoder = diagonalLowerMotor.getEncoder();

    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDiagnostics);
    indexerLayout = driverTab.getLayout("Indexers", BuiltInLayouts.kGrid).withPosition(Constants.indexerIndex, 0)
        .withSize(1, 5);
    indexerLayout.addBoolean("Lower Cargo Present", this::getLowerBB);
    indexerLayout.addNumber("Diagonal Lower RPM", this::getLowerRPM);
    indexerLayout.addNumber("Diagonal Upper RPM", this::getUpperRPM);

    if (Robot.isSimulation()) {
      lowerDiagMotorSim = new FlywheelSim(lowerDiagIndexerGearBox, 1, 1);
      upperDiagMotorSim = new FlywheelSim(upperDiagIndexerGearBox, 1, 1);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runWithVelocityControl() {
    lowerPIDController.setFF(IndexerConstants.DiagonalLower.kS / indexerSetPoint + IndexerConstants.DiagonalLower.kV);
    lowerPIDController.setReference(indexerSetPoint, CANSparkMax.ControlType.kVelocity);

    upperPIDController.setFF(IndexerConstants.DiagonalUpper.kS / indexerSetPoint + IndexerConstants.DiagonalUpper.kV);
    upperPIDController.setReference(indexerSetPoint, CANSparkMax.ControlType.kVelocity);
    if (Robot.isSimulation()) {
      diagonalLowerMotor.set(indexerSetPoint/5800);
      diagonalUpperMotor.set(indexerSetPoint/5800);
    }
  }

  public void setDiagonalIndexerPoint(double setPoint) {
    indexerSetPoint = setPoint;
  }

  public void setDiagonalIndexer(double speed) {
    diagonalLowerMotor.set(speed);
    diagonalUpperMotor.set(speed);
  }

  public boolean isDiagonalIndexerRunning() {
    return Math.abs(diagonalLowerMotor.get()) > 0;
  }

  public boolean getLowerBB() {
    return !lowerBB.get();
  }

  public double getUpperRPM() {
    return upperEncoder.getVelocity();
  }

  public double getLowerRPM() {
    return lowerEncoder.getVelocity();
  }

  @Override
  public void simulationPeriodic() {
    lowerDiagMotorSim.setInput(diagonalLowerMotor.get() * RobotController.getBatteryVoltage());
    upperDiagMotorSim.setInput(diagonalUpperMotor.get() * RobotController.getBatteryVoltage());
    // Next, we update it. The standard loop time is 20ms.
    lowerDiagMotorSim.update(0.020);
    upperDiagMotorSim.update(0.020);

    diagLowerIndexerDial.setAngle(180 * diagonalLowerMotor.get());
    diagUpperIndexerDial.setAngle(180 * diagonalUpperMotor.get());
  }
}
