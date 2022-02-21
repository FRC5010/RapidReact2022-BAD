// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class ShooterConstants {

    // flywheel values

    // pid values
    public static double kPC = 0.26749;
    public static double kIC = 0;
    public static double kDC = 0;

    public static double kP = kPC / 60 / 10;
    public static double kI = kIC / 60;
    public static double kD = kDC / 60;


    // characterized values
    public static double kSC = 0.11361;
    public static double kVC = 0.13197;
    public static double kAC = 0.0066214;

    public static double kS = kSC / 12;
    public static double kV = kVC / 60 / 1 / (12 - kS);
    public static double kA = kAC / 60 / 1 / (12 - kS);

    //Defaults
    public static double defaultFlyWheelRPM = 1000;
    public static double lowRPM = 1000;
    public static double highRPM = 2300;
    public static double feederWheelPower = 0.8;
    public static double indexerPow = 1;
    public static double changeSetPoint = 25;

    public static class HoodConstants{
        public static double kPC = 2.3541;
        public static double kIC = 0;
        public static double kDC = 0;

        public static double kP = kPC / 60;
        public static double kI = kIC / 60;
        public static double kD = kDC / 60;

        public static double kSC = 0.35779;
        public static double kVC = 1.2718;
        public static double kAC = 0.11975;

        public static double kS = kSC / 12;
        public static double kV = kVC / 60 / 1 / (12 - kS);
        public static double kA = kAC / 60 / 1 / (12 - kS);

        public static double hoodInc = 1;
        public static double hoodTolerance = 0.5;
        public static double shootingHoodTolerance = 2.0;
        public static double hoodMaxPos = 40;
        public static double defaultHoodPoint = 30;
        public static double lowHood = 30;
        public static double highHood = 12;
        public static double manualPow = 0.25;
    }

    public static class TurretConstants{
        public static double limitPow = 0.75;

        public static double kPVision = 0.007;
        public static double kDVision = 0.9;

        public static double kPEncoder = 0.02;

        public static double leftLimit = -5;
        public static double rightLimit = 5;
    }
}

