/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class DriveConstants {
    public static double throttleFactor = 0.7;    
    public static double steerFactor = 0.6;    
    public static double drivingAdjustment = 0.05;
    public static int driveInversion = 1;
    
    public static final double ksVolts = 0.18974; //0.1994; //0.2543;
    public static final double kvVoltSecondsPerMeter = 1.3568; // 1.3795; //1.1055;
    public static final double kaVoltSecondsSquaredPerMeter = 0.24208; //0.24467;//0.23628;
    // 2.5 original
    public static final double kPDriveVel = 1.8411; //1.8721; //2.6535; //3.0022;

    public static final double kTrackwidthMeters = 0.616;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 72 * (.0254); // converting from in/s to m/s
    public static final double kMaxAccelerationMetersPerSecondSquared = 60 * (.0254); // converting from in/s/s to m/s/s
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double wheelDiameter = 0.5 * (0.3048); // 12 * 0.0254 - feet to meters
    public static final double motorRotationsPerWheelRotation = 10.71; // i.e. gear ratio
    public static final double pulsesPerMotorRotation = 1.0; // Encoder PPR
    public static final double pulsesPerWheelRotation = pulsesPerMotorRotation * motorRotationsPerWheelRotation;

    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double distancePerPulse = wheelCircumference / pulsesPerWheelRotation;
    public static final double rpmToMetersPerSec = distancePerPulse / 60; // diameter divided by secs-per-minute

    public static final boolean gyroReversed = true;

    public static final boolean leftReversed = false;
    public static final boolean rightReversed = false;
    public static final double leftFudgeFactor = .98;
    public static final double rightFudgeFactor = .982;

    public static final double leftDistanceConv = distancePerPulse * (leftReversed ? -1.0 : 1.0) * leftFudgeFactor;
    public static final double rightDistanceConv = distancePerPulse * (rightReversed ? -1.0 : 1.0) * rightFudgeFactor;
    public static final double leftVelocityConv = rpmToMetersPerSec * (leftReversed ? -1.0 : 1.0) * leftFudgeFactor;
    public static final double rightVelocityConv = rpmToMetersPerSec * (rightReversed ? -1.0 : 1.0) * rightFudgeFactor;

    public static final double kTurnP = 0.5 / 180; // max power / max error
    public static final double kTurnD = kTurnP * 0.1; 
    public static final double minTurn = .3;

    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics, 10);

    public static final TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
            DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(DriveConstants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    // set reversed allows robot to go backwards
    public static final TrajectoryConfig backwardsConfig = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
            DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(DriveConstants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(true);

    public static final TrajectoryConfig pickUpBallConfig = new TrajectoryConfig(1, 1)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint)
            .setReversed(true);

    public static Trajectory driveOffInitLine = TrajectoryGenerator
        .generateTrajectory(List.of(
                new Pose2d(0, 0, new Rotation2d(0)), 
                new Pose2d(-2, 0, new Rotation2d(0))
            ), backwardsConfig);

    public static final Trajectory moveToTrenchPickUp3 = TrajectoryGenerator
    .generateTrajectory(List.of(
        new Pose2d(0, 0, new Rotation2d(0)), 
        new Pose2d(-5, 0, new Rotation2d(0))
    ), pickUpBallConfig);

    public static final Trajectory moveToShoot3 = TrajectoryGenerator
            .generateTrajectory(List.of(
                new Pose2d(7.5, -0.72, new Rotation2d(0)), 
                 new Pose2d(5,-2.4, new Rotation2d(0)),
                new Pose2d(4, -2.4, new Rotation2d(-60))
            ), config);

    public static final Trajectory pickUp2 = TrajectoryGenerator
            .generateTrajectory(List.of(
                new Pose2d(0, 0, new Rotation2d(0)), 
                new Pose2d(-3.5, 0, new Rotation2d(0))
            ), pickUpBallConfig);

        public static final Trajectory moveForward = TrajectoryGenerator
        .generateTrajectory(List.of(
                new Pose2d(-3.5, 0, new Rotation2d(0)), 
                new Pose2d(-2,.5, new Rotation2d(0)),
                new Pose2d(0, .5, new Rotation2d(Math.toRadians(0)))
            ), config);
}
