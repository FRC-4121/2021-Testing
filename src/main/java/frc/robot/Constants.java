/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**  
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Controller ports
    public static final int XBOX_PORT = 0;
    public static final int LEFT_JOY_PORT = 1;
    public static final int RIGHT_JOY_PORT = 2;
    public static final int GAMEPAD_PORT = 3;

    public static final class DrivetrainConstants {

        //0 is West Coast, 1 is Mecanum (simple switching not currently supported)
        public static final int DRIVETRAIN_TYPE = 0;

        //For West Coast, indicates the number of motors per robot side.  ONLY 2 OR 3!!!
        public static final int MOTOR_COUNT = 2;

        //Motor Controller IDs
        public static final int LEFT_MOTOR_1 = 3;
        public static final int LEFT_MOTOR_2 = 6;
        public static final int LEFT_MOTOR_3 = -1;
        public static final int RIGHT_MOTOR_1 = 1;
        public static final int RIGHT_MOTOR_2 = 5;
        public static final int RIGHT_MOTOR_3 = -1;

        //Direction Control
        public static double DIRECTION_MULTIPLIER = 1;

        //PID Constants
        public static final double kP_Straight = 10;
        public static final double kI_Straight = 0.0;
        public static final double kD_Straight = 0.0;
        public static final double kP_Turn = 0;
        public static final double kI_Turn = 0.0;
        public static final double kD_Turn = 0.0;

        //Speeds
        public static final double kAutoDriveSpeed = 0.8;
        public static final double kAutoTurnSpeed = 0.5;

        //Tolerances
        public static final double kTurnAngleTolerance = 0.1;
        public static final double kAutoDistanceTolerance = 1;

        //Drivetrain config constants
        public static final double kWheelDiameter = 6.0;
        public static final double kNEOEncoderPPR = 42;
        public static final double kToughboxMiniGearRatio = 10.71;
        public static final double AUTO_ENCODER_REVOLUTION_FACTOR = 10.71;//needs to be determined by testing.

    }
}
