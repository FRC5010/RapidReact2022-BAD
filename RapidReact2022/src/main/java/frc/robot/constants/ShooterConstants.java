// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class ShooterConstants {

    // flywheel values
    public static final double highHood = HoodConstants.highHood;  
    public static final double highRPM = 2325;
  
    // funny array to store flywheel rpm and hood position to index, index values represent feet, ex index 0 is 0ft

    // new array of 3/17/2022 with foam
    //public static final double[] flyWheelRPM = {highRPM,highRPM, highRPM, highRPM ,2225,2250,2325,2425, 2500, 2600, 2650, 2800, 3150, 3150, 3150,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM};
    //public static final double[] hoodPosition = {highHood,highHood,highHood,highHood,15.0,17.0,19.2,20.2, 22.9, 24.8, 28.2, 30.0, 37.0, 37.0, 37.0,highHood,highHood,highHood,highHood,highHood,highHood,highHood};
    // 4(48) feet = 2225, 15
    // 5(60) feet = 2250, 17
    // 6(72) feet = 2325, 19.2
    // 7(84) feet = 2425, 20.2
    // 8(96) feet = 2500, 22.9
    // 9(108) feet = 2600, 24.8
    // 10(120) feet = 2650, 28.2
    // 11(132) feet = 2800, 30.0
    // 12(144) feet = 3150, 37.0
    // 13(156) feet = 3150, 37.0
    // 14(168) feet = 3150, 37.0
    // 15(180) feet =
    // 16(192) feet = 
    // 17(204) feet =

    
    // 4(48) feet =
    private static double rpm4 = 2425;
    private static double hood4 = 16.9;
    // 5(60) feet = 
    private static double rpm5 = 2500;
    private static double hood5 = 18.2;
    // 6(72) feet = 2325, 19.2
    private static double rpm6 = 2575;
    private static double hood6 = 20.5;
    // 7(84) feet = 2425, 20.2
    private static double rpm7 = 2700;
    private static double hood7 = 20.7;
    // 8(96) feet = 2500, 22.9
    private static double rpm8 = 2775;
    private static double hood8 = 22.8;
    ;
    // 9(108) feet = 2600, 24.8
    private static double rpm9 = 2975;
    private static double hood9 = 25.4;
    // 10(120) feet = 2650, 28.2
    private static double rpm10 = 3075;
    private static double hood10 = 27.8;
    // 11(132) feet = 2800, 30.0
    private static double rpm11 = 3175;
    private static double hood11 = 29.6;
    // 12(144) feet = 3150, 37.0
    private static double rpm12 = 3775;
    private static double hood12 = 38.5;
    // 13(156) feet = 3150, 37.0
    // 14(168) feet = 3150, 37.0
    // 15(180) feet =
    // 16(192) feet = 
    // 17(204) feet =
    public static final double[] flyWheelRPM = {highRPM,highRPM, highRPM, highRPM ,rpm4,rpm5,rpm6,rpm7, rpm8, rpm9, rpm10, rpm11, rpm12, rpm12, rpm12,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM,highRPM};
    public static final double[] hoodPosition = {highHood,highHood,highHood,highHood,hood4,hood5,hood6,hood7, hood8, hood9, hood10, hood11, hood12, hood12, hood12,highHood,highHood,highHood,highHood,highHood,highHood,highHood};
    
        // pid values
    public static double kPC = 0.2;//0.11889;
    public static double kIC = 0.000005;
    public static double kDC = 0.5;

    public static double kP = kPC / 60 / 10;
    public static double kI = kIC / 60;
    public static double kD = kDC / 60;


    // characterized values
    public static double kSC = 0.11449; // 0.11361;
    public static double kVC = 0.12735; // 0.13197;
    public static double kAC = 0.00649; // 0.0066214;

    public static double kS = kSC / 12;
    public static double kV = kVC / 60 / 1 / (12 - kS);
    public static double kA = kAC / 60 / 1 / (12 - kS);

    //Defaults
    public static double defaultFlyWheelRPM = 1000;
    public static double lowRPM = 900;
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
        public static double kPC = 0.58908; // 2.3541;
        public static double kIC = 0;
        public static double kDC = 0;

        public static double kP = kPC / 60;
        public static double kI = kIC / 60;
        public static double kD = kDC / 60;

        public static double kSC = 0.22225;  // 0.35779
        public static double kVC = 0.59444;  // 1.2718
        public static double kAC = 0.034764; // 0.11975

        public static double kS = kSC / 12;
        public static double kV = kVC / 60 / 1 / (12 - kS);
        public static double kA = kAC / 60 / 1 / (12 - kS);

        public static double hoodInc = 1;
        public static double hoodTolerance = 0.5;
        public static double shootingHoodTolerance = 2.0;
        public static double hoodMaxPos = 40;
        public static double defaultHoodPoint = 30;
        public static double lowHood = 30;
        public static double highHood = 10.38; //4 is what this was during 1st comp
        public static double manualPow = 0.25;
    }

    public static class TurretConstants{
        public static double limitPow = 0.75;

        public static double seekSpeed = 0.15;

        public static double kS = 0.02;

        public static double kPVision = 0.0065; //0.004;
        public static double kDVision = 2000; //850;

        public static double kPEncoder = 0.15;

        // 3/26/2022, changed limits of turret to keep static hooks up
        // old values for full range are commented outs
        public static double leftLimit = -0.35; //-1.8;
        public static double rightLimit = 0.35; //1.3;

        public static double onTargetLowLimit = 2; 
        public static double onTargetHighLimit = 5; 
    }
}

