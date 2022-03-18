// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class ShooterConstants {

    // flywheel values

    // pid values
    public static double kPC = 0.2;//0.11889;
    public static double kIC = 0.000005;
    public static double kDC = 0.5;

    public static double kP = kPC / 60 / 10;
    public static double kI = kIC / 60;
    public static double kD = kDC / 60;


    // characterized values
    public static double kSC = 0.084437; // 0.11361;
    public static double kVC = 0.12852; // 0.13197;
    public static double kAC = 0.00671; // 0.0066214;

    public static double kS = kSC / 12;
    public static double kV = kVC / 60 / 1 / (12 - kS);
    public static double kA = kAC / 60 / 1 / (12 - kS);

    //Defaults
    public static double defaultFlyWheelRPM = 1000;
    public static double lowRPM = 900;
    public static double highRPM = 2075;
    public static double indexerPow = 1;
    public static double changeSetPoint = 25;
    public static double shotAdjustment = 0;

    public static class FeederConstants{
        public static double feederWheelPower = 1;
        public static double feederWheelRPM = 4075;
        // pid values
        public static double kPC = 0.043836; //0.044696;
        public static double kIC = 0;
        public static double kDC = 0;

        public static double kP = kPC / 60 / 10;
        public static double kI = kIC / 60;
        public static double kD = kDC / 60;


        // characterized values
        public static double kSC = 0.086624; //0.08335;
        public static double kVC = 0.12209; //0.12194;
        public static double kAC = 0.0022249; //0.0022566;

        public static double kS = kSC / 12;
        public static double kV = kVC / 60 / 1 / (12 - kS);
        public static double kA = kAC / 60 / 1 / (12 - kS);
    }

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
        public static double highHood = 10; //4 is what this was during 1st comp
        public static double manualPow = 0.25;
    }

    public static class TurretConstants{
        public static double limitPow = 0.75;

        public static double kPVision = 0.007;
        public static double kDVision = 1.05;

        public static double kPEncoder = 0.02;

        public static double leftLimit = -15;
        public static double rightLimit = 10;
    }
}

