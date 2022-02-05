/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.mechanisms.Drive;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Pose;

/**
 * An example command that uses an example subsystem.
 */
public class RamseteFollower extends RamseteCommand {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Trajectory trajectory;
  private Timer timer;
  private double accXDiff = 0;
  private double accYDiff = 0;
  private double totalDistance = 0;
  private Pose pose;
  private boolean reset;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RamseteFollower(Trajectory trajectory, boolean reset) {
    super(trajectory, Drive.robotPose::getPose,
      new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics, Drive.robotPose::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
      Drive.driveTrain::tankDriveVolts, Drive.driveTrain);

    this.pose = Drive.robotPose;
    timer = new Timer();
    this.reset = reset;
    this.trajectory = trajectory;
    State finalState = trajectory.sample(trajectory.getTotalTimeSeconds());
    totalDistance = finalState.poseMeters.getTranslation().getX();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void reset() {
    pose.resetOdometry(trajectory.getInitialPose());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    timer.reset();
    timer.start();

    if(reset){
      reset();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    State expState = trajectory.sample(timer.get());
    Pose2d expectedPose = expState.poseMeters;
    DifferentialDriveOdometry odometer = Drive.robotPose.getOdometry();
    Pose2d actualPose = odometer.getPoseMeters();
    double actualX = actualPose.getTranslation().getX();
    double actualY = actualPose.getTranslation().getY();
    double expectedX = expectedPose.getTranslation().getX();
    double expectedY = expectedPose.getTranslation().getY();

    accXDiff += Math.abs(expectedX - actualX);
    accYDiff += Math.abs(expectedY - actualY);
    // SmartDashboard.putNumber("Expected xpos", expectedPose.getTranslation().getX());
    // SmartDashboard.putNumber("Expected Y pos", expectedPose.getTranslation().getY());
    // SmartDashboard.putNumber("Expected heading", expectedPose.getRotation().getDegrees());
    // SmartDashboard.putNumber("Expected vel m/s", expState.velocityMetersPerSecond);

    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    timer.stop();
    // SmartDashboard.putNumber("Avg X Diff per meter ", accXDiff / totalDistance);
    // SmartDashboard.putNumber("Avg Y Diff per meter", accYDiff / totalDistance);

    System.out.println("Ramsete ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
