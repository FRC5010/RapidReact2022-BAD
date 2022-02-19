/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.HIDType;

/**
 * Add your docs here.
 */
public class ControlConstants {
    static enum ButtonNums {
        NO_BUTTON, A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON, LEFT_BUMPER, RIGHT_BUMPER, START_BUTTON,
        BACK_BUTTON,LEFT_STICK_BUTT, RIGHT_STICK_BUTT;
    }
    static enum AxisNums {
    LEFT_X, LEFT_Y, 
     L_TRIGGER, 
     R_TRIGGER, 
    RIGHT_X, RIGHT_Y
    }
    static enum POVDirs {
        UP, RIGHT, DOWN, LEFT 
    }
    static enum Motors{
        NO_MOTOR, M1, M2, M3, M4, M5, M6, M7, M8, M9, M10, M11, M12, M13, M14, M15, M16;
    }
    static enum DioPorts {
        Port0, Port1, Port2, Port3, Port4, Port5, Port6, Port7, Port8, Port9;
    }
    //DIO Ports
    public static int limitSwitch = DioPorts.Port0.ordinal();
    public static int BB1 = DioPorts.Port0.ordinal();
    //public static int BB2 = DioPorts.Port2.ordinal();

    /*
    Motors
        M1: Left Drive
        M2: Left Drive
        M3: Left Drive
        M4: Right Drive
        M5: Right Drive
        M6: Right Drive
        M7: Index Motor
        M8: Index Motor
        M9: Intake
        M10: Hood
        M11: Feeder Wheel
        M12: Left Flywheel
        M13: Right Flywheel
        M14: Turret
        M15: Left Climb
        M16: Right Climb
    */

    // Motor
    public static int leftDrive1M = Motors.M1.ordinal();
    public static int leftDrive2M = Motors.M2.ordinal();
    public static int leftDrive3M = Motors.M3.ordinal();
    public static int rightDrive1M = Motors.M4.ordinal();
    public static int rightDrive2M = Motors.M5.ordinal();
    public static int rightDrive3M = Motors.M6.ordinal();
    public static int intakeM = Motors.M9.ordinal();
    public static int lowerIndexM = Motors.M8.ordinal();
    public static int upperIndexM = Motors.M7.ordinal();
    public static int hoodM = Motors.M10.ordinal();
    public static int feederWheelM = Motors.M11.ordinal();
    public static int leftFlyWheelM = Motors.M12.ordinal();
    public static int rightFlyWheelM = Motors.M13.ordinal();
    public static int turretM = Motors.M14.ordinal();
    public static int leftClimbM = Motors.M15.ordinal();
    public static int rightClimbM = Motors.M16.ordinal();

    // Driver
    public static int driverJoystick = 0;
    public static int throttle = AxisNums.LEFT_Y.ordinal();
    public static int steer = AxisNums.RIGHT_X.ordinal();
    public static int steerY = AxisNums.RIGHT_Y.ordinal();
    //public static int winch1Axis = AxisNums.L_TRIGGER.ordinal();
    //public static int winch2Axis = AxisNums.R_TRIGGER.ordinal();

    public static int manualShootButton = ButtonNums.A_BUTTON.ordinal();
    public static int shooterAimButton = ButtonNums.B_BUTTON.ordinal();
    public static int tarmatShoot = ButtonNums.RIGHT_BUMPER.ordinal();
    //public static int rotationControl = ButtonNums.X_BUTTON.ordinal();
    //public static int positionControl = ButtonNums.Y_BUTTON.ordinal();
    public static int upperFender = ButtonNums.LEFT_BUMPER.ordinal();
    public static int toggleIntake = ButtonNums.RIGHT_BUMPER.ordinal();
    public static int calibrate = ButtonNums.START_BUTTON.ordinal(); 

    //public static int startClimb = ButtonNums.BACK_BUTTON.ordinal();
    //public static int toggleDrive = ButtonNums.LEFT_STICK_BUTT.ordinal();
    public static int toggleLed = ButtonNums.RIGHT_STICK_BUTT.ordinal();
    
    

    public static int incThrottleFactor = POVDirs.UP.ordinal() * 90;
    public static int decThrottleFactor = POVDirs.DOWN.ordinal() * 90;
    public static int decSteerFactor = POVDirs.LEFT.ordinal() * 90;
    public static int incSteerFactor = POVDirs.RIGHT.ordinal() * 90;

    //Operator
    public static int operatorJoystick = 1;
    public static int operatorLeftY = AxisNums.LEFT_Y.ordinal(); // Implement
    public static int operatorRightY = AxisNums.RIGHT_Y.ordinal(); // Implement
    public static int operatorRightX = AxisNums.RIGHT_X.ordinal();
    public static int outtakeAxis = AxisNums.L_TRIGGER.ordinal() ;
    public static int intakeAxis = AxisNums.R_TRIGGER.ordinal();

    public static int indexerDown = ButtonNums.A_BUTTON.ordinal();
    public static int hoodDown = ButtonNums.B_BUTTON.ordinal(); 
    public static int indexerUp = ButtonNums.X_BUTTON.ordinal(); // not used
    public static int hoodUp = ButtonNums.Y_BUTTON.ordinal(); 
    public static int fenderButton = ButtonNums.LEFT_BUMPER.ordinal();
    public static int launchButton = ButtonNums.RIGHT_BUMPER.ordinal();
    //public static int startFlywheel = ButtonNums.LEFT_STICK_BUTT.ordinal();
    //public static int stopFlywheel = ButtonNums.RIGHT_STICK_BUTT.ordinal();
    //public static int overrideIntake = ButtonNums.START_BUTTON.ordinal();

    public static int defaultShoot = ButtonNums.START_BUTTON.ordinal();
    public static int selCamera2 = ButtonNums.BACK_BUTTON.ordinal();

    public static int incShooter = POVDirs.UP.ordinal() * 90;
    public static int decShooter = POVDirs.DOWN.ordinal() * 90;
    //public static int spinnerOverrideButtonLow = POVDirs.RIGHT.ordinal() * 90;
    //public static int spinnerOverrideButtonHigh = POVDirs.LEFT.ordinal() * 90;

    // Shuffleboard constants
    public static String SBTabDriverDisplay = "Driver Display";
    public static String SBTabVisionDisplay = "Vision Display";
    public static String SBTabDiagnostics = "Diagnostics";
    public static int shooterColumn = 0;
    public static int hoodColumn = 2;

    public static int driverColumn = 4;
    public static int autoColumn = 4;
    public static int limelightColumn = 0;
    public static int intakeVisionColumn = 0;
    public static int shooterVisionColumn = 6;

    public static int autoNavButton = ButtonNums.X_BUTTON.ordinal();
    public static int driveTrainCurrentLimit = 38;
    public static int babyNeoCurrentLimit = 30;
    public static int neoCurrentLimit = 38;
    
    public static boolean setupSingleDriver(Joystick operator){
        if(operator.getType() == HIDType.kUnknown){
            throttle = AxisNums.LEFT_Y.ordinal();
            steer = AxisNums.RIGHT_X.ordinal();
        
            //startClimb = ButtonNums.LEFT_BUMPER.ordinal();
        
            launchButton = ButtonNums.A_BUTTON.ordinal();
            autoNavButton = ButtonNums.X_BUTTON.ordinal();
            return true;
        }
        return false;
    }
}
