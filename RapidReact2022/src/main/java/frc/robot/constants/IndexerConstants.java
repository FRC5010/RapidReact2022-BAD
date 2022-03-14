// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class IndexerConstants {
    

    public static class VerticalLong{
        // pid values
        public static double kPC = 0;
        public static double kIC = 0;
        public static double kDC = 0;

        public static double kP = kPC / 60 / 10;
        public static double kI = kIC / 60;
        public static double kD = kDC / 60;

        public static double kSC = 0;
        public static double kVC = 0;
        public static double kAC = 0;

        public static double kS = kSC / 12;
        public static double kV = kVC / 60 / 1 / (12 - kS);
        public static double kA = kAC / 60 / 1 / (12 - kS);
    }

    public static class VerticalShort{
        // pid values
        public static double kPC = 0;
        public static double kIC = 0;
        public static double kDC = 0;

        public static double kP = kPC / 60 / 10;
        public static double kI = kIC / 60;
        public static double kD = kDC / 60;

        public static double kSC = 0;
        public static double kVC = 0;
        public static double kAC = 0;

        public static double kS = kSC / 12;
        public static double kV = kVC / 60 / 1 / (12 - kS);
        public static double kA = kAC / 60 / 1 / (12 - kS);
    }

    public static class DiagonalLower{
        // pid values
        public static double kPC = 0;
        public static double kIC = 0;
        public static double kDC = 0;

        public static double kP = kPC / 60 / 10;
        public static double kI = kIC / 60;
        public static double kD = kDC / 60;

        public static double kSC = 0;
        public static double kVC = 0;
        public static double kAC = 0;

        public static double kS = kSC / 12;
        public static double kV = kVC / 60 / 1 / (12 - kS);
        public static double kA = kAC / 60 / 1 / (12 - kS);
    }

    public static class DiagonalUpper{
        // pid values
        public static double kPC = 0;
        public static double kIC = 0;
        public static double kDC = 0;

        public static double kP = kPC / 60 / 10;
        public static double kI = kIC / 60;
        public static double kD = kDC / 60;

        public static double kSC = 0;
        public static double kVC = 0;
        public static double kAC = 0;

        public static double kS = kSC / 12;
        public static double kV = kVC / 60 / 1 / (12 - kS);
        public static double kA = kAC / 60 / 1 / (12 - kS);
    }


    public static Color kBlueTarget = new Color(0.143, 0.427, 0.429);
   // public static Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    public static Color kRedTarget = new Color(0.561, 0.232, 0.114);
    // public static Color kYellowTarget = new Color(0.361, 0.524, 0.113);
}
