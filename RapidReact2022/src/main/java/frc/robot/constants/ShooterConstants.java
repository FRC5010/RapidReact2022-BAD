// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class ShooterConstants {
    // pid values
    public static double flyWheelP = 0;
    public static double flyWheelI = 0;
    public static double flyWheelD = 0;

    // characterized values
    public static double kSC = 0.11361;
    public static double kVC = 0.13197;
    public static double kAC = 0.0066214;

    public static double kS = kSC / 12;
    public static double kV = kVC / 60 / 1 / (12 - kS);
    public static double kA = kAC / 60 / 1 / (12 - kS);

    //Defaults
    public static double defaultFlywheelRPM = 1000;
    public static double feederWheelPower = .5;
    public static double indexerPow = 0.6;
}
