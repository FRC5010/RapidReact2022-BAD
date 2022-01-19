/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.DriveConstants;

/**
 * Add your docs here.
 */
public class Pose extends SubsystemBase {

    public double getEncoderDistance(RelativeEncoder encoder, double conversion) {
        return encoder.getPosition() * conversion;
    }

    public double getEncoderVel(RelativeEncoder encoder, double conversion) {
        return encoder.getVelocity() * conversion;
    }

    // The robot's drive
    // private final DifferentialDrive m_drive = new DifferentialDrive(leftMaster1,
    // rightMaster1);

    // The left-side drive encoder
    public final RelativeEncoder leftEncoder;

    // The right-side drive encoder
    public final RelativeEncoder rightEncoder;

    // The gyro sensor
    public final AHRS gyro = new AHRS(Port.kUSB1);

    // Odometry class for tracking robot pose

    private final DifferentialDriveOdometry odometry;

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    public Pose(RelativeEncoder leftEncoder, RelativeEncoder rightEncoder) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        gyro.reset();
        
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        resetEncoders();
        ShuffleboardLayout poseLayout = Shuffleboard.getTab(ControlConstants.SBTabDiagnostics).getLayout("Pose",
                BuiltInLayouts.kList);
        poseLayout.addNumber("left encoder distance",
                () -> getEncoderDistance(leftEncoder, DriveConstants.leftDistanceConv));
        poseLayout.addNumber("right encoder distance",
                () -> getEncoderDistance(rightEncoder, DriveConstants.rightDistanceConv));
        poseLayout.addNumber("gyro heading", () -> getHeading());

        poseLayout.addNumber("Robot X pos", () -> odometry.getPoseMeters().getTranslation().getX());
        poseLayout.addNumber("Robot Y pos", () -> odometry.getPoseMeters().getTranslation().getY());
        poseLayout.addNumber("Robot Heading", () -> odometry.getPoseMeters().getRotation().getDegrees());
        poseLayout.addNumber("Robot Velocity", () -> leftEncoder.getVelocity());

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
        return new DifferentialDriveWheelSpeeds(getEncoderVel(leftEncoder, DriveConstants.leftVelocityConv),
                getEncoderVel(rightEncoder, DriveConstants.rightVelocityConv));
    }

    @Override
    public void periodic() {
        posePeriodic();
    }

    public void posePeriodic() {
        odometry.update(Rotation2d.fromDegrees(getHeading()),
                getEncoderDistance(leftEncoder, DriveConstants.leftDistanceConv),
                getEncoderDistance(rightEncoder, DriveConstants.rightDistanceConv));
        // SmartDashboard.putNumber("left encoder position", leftEncoder.getPosition());
        // SmartDashboard.putNumber("right encoder position",
        // rightEncoder.getPosition());

        // SmartDashboard.putNumber("left velocity", getEncoderVel(leftEncoder,
        // DriveConstants.leftVelocityConv));
        // SmartDashboard.putNumber("right velocity", getEncoderVel(rightEncoder,
        // DriveConstants.rightVelocityConv));
        // SmartDashboard.putNumber("left raw velocity", leftEncoder.getVelocity());
        // SmartDashboard.putNumber("right raw velocity", rightEncoder.getVelocity());
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
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    // public void arcadeDrive(double fwd, double rot) {
    // m_drive.arcadeDrive(fwd, rot);
    // }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (getEncoderDistance(leftEncoder, DriveConstants.leftDistanceConv)
                + getEncoderDistance(rightEncoder, DriveConstants.rightDistanceConv)) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public RelativeEncoder getRightEncoder() {
        return rightEncoder;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained //
     */
    // public void setMaxOutput(double maxOutput) {
    // RobotContainer.driveTrain.setMaxOutput(maxOutput);
    // }

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