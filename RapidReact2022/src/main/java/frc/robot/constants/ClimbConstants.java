// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class ClimbConstants {
    public static double climbSpeedUp = 1;
    public static double climbSpeedDown = -1;
    public static double encoderMax = 0;

    // 3/26/2022 new climb limits changing gearboxes at tippie
    public static double climbBothMax = 149;//106;
    public static double climbRightMax = climbBothMax; // not used
    public static double climbLeftMax = climbBothMax; // not used
    
    public static double climbStaticMax = 0; //
    public static double calibrateDown = -0.6;

}
