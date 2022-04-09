// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class IndexerConstants {
    // verticals are 10:1
    // diagonals are 7:1

    public static double indexerRPM = 10000;
    public static double diagIndexerRPM = 10000 / 2;

    public static class VerticalLong{
        // pid values
        public static double kPC = 0.20779;
        public static double kIC = 0;
        public static double kDC = 0;

        public static double kP = kPC / 60 / 10;
        public static double kI = kIC / 60;
        public static double kD = kDC / 60;

        public static double kSC = 0.22328;
        public static double kVC = 0.61725;
        public static double kAC = 0.010692;

        public static double kS = kSC / 12;
        public static double kV = kVC / 60 / 1 / (12 - kS);
        public static double kA = kAC / 60 / 1 / (12 - kS);
    }

    public static class VerticalShort{
        // pid values
        public static double kPC = 0.38423;
        public static double kIC = 0;
        public static double kDC = 0;

        public static double kP = kPC / 60 / 10;
        public static double kI = kIC / 60;
        public static double kD = kDC / 60;

        public static double kSC = 0.1577;
        public static double kVC = 0.63479;
        public static double kAC = 0.018236;

        public static double kS = kSC / 12;
        public static double kV = kVC / 60 / 1 / (12 - kS);
        public static double kA = kAC / 60 / 1 / (12 - kS);
    }

    public static class DiagonalUpper{
        // pid values
        public static double kPC = 0.54279;
        public static double kIC = 0;
        public static double kDC = 1.3;

        public static double kP = kPC / 60 / 10;
        public static double kI = kIC / 60;
        public static double kD = kDC / 60;

        public static double kSC = 0.3863;
        public static double kVC = 0.4689;
        public static double kAC = 0.03878;

        public static double kS = kSC / 12;
        public static double kV = kVC / 60 / 1 / (12 - kS);
        public static double kA = kAC / 60 / 1 / (12 - kS);
    }

    public static class DiagonalLower{
        // pid values
        public static double kPC = 0.32996;
        public static double kIC = 0;
        public static double kDC = 0.5;

        public static double kP = kPC / 60 / 10;
        public static double kI = kIC / 60;
        public static double kD = kDC / 60;

        public static double kSC = 0.27991;
        public static double kVC = 0.44398;
        public static double kAC = 0.016099;

        public static double kS = kSC / 12;
        public static double kV = kVC / 60 / 1 / (12 - kS);
        public static double kA = kAC / 60 / 1 / (12 - kS);
    }


    public static Color kBlueTarget = new Color(0.143, 0.427, 0.429);
   // public static Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    public static Color kRedTarget = new Color(0.561, 0.232, 0.114);
    // public static Color kYellowTarget = new Color(0.361, 0.524, 0.113);
}
