/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.GenericEncoder;
import frc.robot.FRC5010.GenericGyro;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.mechanisms.Drive;

/**
 * Add your docs here.
 */
public class Pose extends SubsystemBase {
    private final Field2d m_fieldSim = new Field2d();

    public double getEncoderDistance(GenericEncoder encoder) {
        return encoder.getPosition();
    }

    public double getEncoderVel(GenericEncoder encoder) {
        return encoder.getVelocity();
    }

    // The robot's drive
    // private final DifferentialDrive m_drive = new DifferentialDrive(leftMaster1,
    // rightMaster1);

    // The left-side drive encoder
    public final GenericEncoder leftEncoder;

    // The right-side drive encoder
    public final GenericEncoder rightEncoder;

    // The gyro sensor
    public final GenericGyro gyro;

    //  I2C.Port.kMXP

    // Odometry class for tracking robot pose

    private final DifferentialDriveOdometry odometry;

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    public Pose(GenericEncoder leftEncoder, GenericEncoder rightEncoder, GenericGyro gyro) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.gyro = gyro;

        gyro.reset();
        
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        resetEncoders();
        ShuffleboardLayout poseLayout = Shuffleboard.getTab(ControlConstants.SBTabDiagnostics).getLayout("Pose",
                BuiltInLayouts.kList);
        poseLayout.addNumber("Left encoder distance",
                () -> getEncoderDistance(leftEncoder));
        poseLayout.addNumber("Right encoder distance",
                () -> getEncoderDistance(rightEncoder));
        poseLayout.addNumber("Gyro heading", () -> getHeading());

        poseLayout.addNumber("Robot X pos", () -> odometry.getPoseMeters().getTranslation().getX());
        poseLayout.addNumber("Robot Y pos", () -> odometry.getPoseMeters().getTranslation().getY());
        poseLayout.addNumber("Robot Heading", () -> odometry.getPoseMeters().getRotation().getDegrees());
        poseLayout.addNumber("Robot Velocity", () -> leftEncoder.getVelocity());

        SmartDashboard.putData("Field", m_fieldSim);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveOdometry getOdometry() {
        return odometry;
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getEncoderVel(leftEncoder),
                getEncoderVel(rightEncoder));
    }

    @Override
    public void periodic() {
        posePeriodic();
        m_fieldSim.setRobotPose(odometry.getPoseMeters());
    }

    public void posePeriodic() {
        odometry.update(Rotation2d.fromDegrees(getHeading()),
                getEncoderDistance(leftEncoder),
                getEncoderDistance(rightEncoder));
    }

      /** Update our simulation. This should be run every robot loop in simulation. */
  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    Drive.m_driveSim.setInputs(
        Drive.lDrive1.get() * RobotController.getInputVoltage(),
        Drive.rDrive1.get() * RobotController.getInputVoltage());
    Drive.m_driveSim.update(0.02);

    leftEncoder.setPosition(Drive.m_driveSim.getLeftPositionMeters());
    leftEncoder.setRate(Drive.m_driveSim.getLeftVelocityMetersPerSecond());
    rightEncoder.setPosition(Drive.m_driveSim.getRightPositionMeters());
    rightEncoder.setRate(Drive.m_driveSim.getRightVelocityMetersPerSecond());
    gyro.setAngle(-Drive.m_driveSim.getHeading().getDegrees());
  }
  
    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {//Rotation2d.fromDegrees(getHeading())
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();;
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (getEncoderDistance(leftEncoder)
                + getEncoderDistance(rightEncoder)) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public GenericEncoder getLeftEncoder() {
        return leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public GenericEncoder getRightEncoder() {
        return rightEncoder;
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.gyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
    }
}