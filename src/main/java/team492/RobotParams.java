/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcPidConveyor;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcPidController.PidCoefficients;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import team492.drivebases.RobotDrive.DriveMode;

/**
 * This class contains parameters and preferences related to all robot operations.
 */
public class RobotParams
{
    public enum RobotType
    {
        NoRobot,
        DifferentialRobot,
        MecanumRobot,
        SwerveRobot,
        ChadRobot
    }   //enum RobotType

    public enum SteerEncoderType
    {
        CANCoder,
        Canandcoder,
        AnalogEncoder
    }   //enum SteerEncoderType

    //
    // Robot preferences.
    //
    public static class Preferences
    {
        // Global config
        public static RobotType robotType                       = RobotType.SwerveRobot;
        public static boolean inCompetition                     = false;
        public static final boolean hybridMode                  = false;
        public static final boolean useTraceLog                 = true;
        // Status Update
        public static final boolean doStatusUpdate              = false;
        public static final boolean showLoopTime                = false;
        public static final boolean showPowerConsumption        = false;
        public static final boolean showDriveBase               = false;
        public static final boolean showPurePursuitDrive        = false;
        public static final boolean showPidDrive                = false;
        public static final boolean showVision                  = false;
        public static final boolean showSubsystems              = false;
        // Inputs
        public static final boolean useDriverXboxController     = true;
        public static final boolean useTankDrive                = false;
        public static final boolean doOneStickDrive             = false;
        public static final boolean useButtonPanels             = false;
        // Sensors
        public static final boolean useNavX                     = true;
        public static final boolean usePdp                      = false;
        public static final boolean usePressureSensor           = false;
        // Vision
        public static final boolean useVision                   = true;
        public static final boolean usePhotonVision             = true;
        public static final boolean usePhotonVisionRaw          = false;
        public static final boolean useOpenCvVision             = false;
        public static final boolean useStreamCamera             = false;
        // Drive Base
        public static final boolean useExternalOdometry         = false;
        public static final boolean useVelocityControl          = false;
        public static final boolean useGyroAssist               = false;
        public static final boolean useAntiTipping              = false;
        // Subsystems
        public static final boolean useSubsystems               = false;
        public static final boolean useIntake                   = false;
        public static final boolean useShooter                  = false;
        public static final boolean useClimber                  = false;
        public static final boolean useDeployer                 = false;
    }   //class Preferences

    public static final String ROBOT_NAME                       = "Robot492";
    public static final String TEAM_FOLDER_PATH                 = "/home/lvuser/trc492";
    public static final String STEER_ZERO_CAL_FILE              = "SteerCalibration.txt";
    public static final double DASHBOARD_UPDATE_INTERVAL        = 0.1;          // in msec
    //
    // Field dimensions in inches.
    //
    public static final double FIELD_LENGTH                     = 54.0*12.0;
    public static final double FIELD_WIDTH                      = 27.0*12.0;
    //
    // Robot dimensions in inches.
    //
    public static final double ROBOT_WIDTH                      = 34.5;     // Frame dimensions, including bumpers.
    public static final double ROBOT_LENGTH                     = 37.0;     // Frame dimensions, including bumpers.

    public static final double ROBOT_WHEELBASE_WIDTH            = 23.25;    // Required by swerve drive base.
    public static final double ROBOT_WHEELBASE_LENGTH           = 23.25;    // Required by swerve drive base.
    //
    // Robot starting positions.
    //
    public static final double STARTPOS_BLUE_Y                  = ROBOT_LENGTH / 2.0;
    public static final double STARTPOS_RED_Y                   = FIELD_LENGTH - STARTPOS_BLUE_Y;
    public static final double STARTPOS_1_X                     = -42.19;
    public static final double STARTPOS_2_X                     = -108.19;
    public static final double STARTPOS_3_X                     = -174.19;
    public static final TrcPose2D STARTPOS_BLUE_1 = new TrcPose2D(STARTPOS_1_X, STARTPOS_BLUE_Y, 180.0);
    public static final TrcPose2D STARTPOS_BLUE_2 = new TrcPose2D(STARTPOS_2_X, STARTPOS_BLUE_Y, 180.0);
    public static final TrcPose2D STARTPOS_BLUE_3 = new TrcPose2D(STARTPOS_3_X, STARTPOS_BLUE_Y, 180.0);
    public static final TrcPose2D STARTPOS_RED_1 = new TrcPose2D(STARTPOS_1_X, STARTPOS_RED_Y, 0.0);
    public static final TrcPose2D STARTPOS_RED_2 = new TrcPose2D(STARTPOS_2_X, STARTPOS_RED_Y, 0.0);
    public static final TrcPose2D STARTPOS_RED_3 = new TrcPose2D(STARTPOS_3_X, STARTPOS_RED_Y, 0.0);
    public static final TrcPose2D[][] startPos =
    {
        {STARTPOS_BLUE_1, STARTPOS_BLUE_2, STARTPOS_BLUE_3},
        {STARTPOS_RED_1, STARTPOS_RED_2, STARTPOS_RED_3}
    };
    //
    // Game element locations and dimensions.
    //

    //
    // Joystick ports.
    //
    public static final int XBOX_DRIVER_CONTROLLER              = 0;
    public static final int JSPORT_DRIVER_LEFTSTICK             = 0;
    public static final int JSPORT_DRIVER_RIGHTSTICK            = 1;
    public static final int JSPORT_OPERATORSTICK                = 2;
    public static final int JSPORT_BUTTON_PANEL                 = 3;
    public static final int JSPORT_SWITCH_PANEL                 = 4;
    //
    // CAN IDs.
    //
    public static final int CANID_LFDRIVE_MOTOR                 = 3;
    public static final int CANID_RFDRIVE_MOTOR                 = 4;
    public static final int CANID_LBDRIVE_MOTOR                 = 5;
    public static final int CANID_RBDRIVE_MOTOR                 = 6;

    // Applicable only for Swerve Drive.
    public static final int CANID_LFSTEER_MOTOR                 = 13;
    public static final int CANID_RFSTEER_MOTOR                 = 14;
    public static final int CANID_LBSTEER_MOTOR                 = 15;
    public static final int CANID_RBSTEER_MOTOR                 = 16;

    // Applicable only for Swerve Drive.
    public static final int CANID_LFSTEER_ENCODER               = 23;
    public static final int CANID_RFSTEER_ENCODER               = 24;
    public static final int CANID_LBSTEER_ENCODER               = 21;//25;
    public static final int CANID_RBSTEER_ENCODER               = 22;//26;

    // Subsystems.
    public static final int CANID_INTAKE_MOTOR                  = 7;
    public static final int CANID_SHOOTER_MOTOR                 = 8;
    public static final int CANID_TILTER_MOTOR                  = 9;
    public static final int CANID_CLIMBER_MOTOR                 = 17;

    public static final int CANID_PCM                           = 30;
    public static final int CANID_PDP                           = 31;
    //
    // PDP Channels.
    //
    public static final int PDP_CHANNEL_LFDRIVE_MOTOR           = 11;
    public static final int PDP_CHANNEL_RFDRIVE_MOTOR           = 5;
    public static final int PDP_CHANNEL_LBDRIVE_MOTOR           = 13;
    public static final int PDP_CHANNEL_RBDRIVE_MOTOR           = 3;
    public static final int PDP_CHANNEL_LFSTEER_MOTOR           = 10;
    public static final int PDP_CHANNEL_RFSTEER_MOTOR           = 6;
    public static final int PDP_CHANNEL_LBSTEER_MOTOR           = 12;
    public static final int PDP_CHANNEL_RBSTEER_MOTOR           = 4;

    public static final int PDP_CHANNEL_ROBORIO                 = 20;
    public static final int PDP_CHANNEL_VRM                     = 18;
    public static final int PDP_CHANNEL_PCM                     = 19;
    public static final int PDP_CHANNEL_RADIO_POE               = 22;
    public static final int PDP_CHANNEL_ETHERNET_SWITCH         = 21;
    public static final int PDP_CHANNEL_LIMELIGHT               = 0;
    public static final int PDP_CHANNEL_LED                     = 14;

    public static final double BATTERY_CAPACITY_WATT_HOUR       = 18.0*12.0;
    //
    // Analog Input ports.
    //
    public static final int AIN_PRESSURE_SENSOR                 = 0;
    public static final int AIN_LFSTEER_ENCODER                 = 0;    // Black
    public static final int AIN_RFSTEER_ENCODER                 = 1;    // Brown
    public static final int AIN_LBSTEER_ENCODER                 = 2;    // Red
    public static final int AIN_RBSTEER_ENCODER                 = 3;    // Orange
    //
    // Digital Input/Output ports.
    //
    public static final int DIO_INTAKE_ENTRY                    = 0;
    public static final int DIO_INTAKE_EXIT                     = 1;

    //
    // PWM channels.
    //
    public static final int NUM_LEDS                            = 60;
    public static final int PWM_CHANNEL_LED                     = 9;
    //
    // Relay channels.
    //

    //
    // Pneumatic channels.
    //

    //
    // Ultrasonic sensors.
    //
    // public static final double SONAR_INCHES_PER_VOLT            = 1.0/0.0098; //9.8mV per inch
    // public static final double SONAR_ERROR_THRESHOLD            = 50.0; //value should not jump 50-in per time slice.

    //
    // Vision subsystem.
    //
    public static class Vision
    {
        public static final int CAMERA_IMAGE_WIDTH              = 320;      // in pixels
        public static final int CAMERA_IMAGE_HEIGHT             = 240;      // in pixels
        // Camera location on robot.
        public static final double CAMERA_Y_OFFSET              = 9.5;      // Inches from the center of the robot
        public static final double CAMERA_X_OFFSET              = 0.0;      // Exactly centered
        public static final double CAMERA_HEIGHT                = 10.75;    // Inches from the floor
        public static final double CAMERA_PITCH                 = -8.0;     // degrees from horizontal
        public static final double CAMERA_YAW                   = -2.9126252095;    // degrees from front
        public static final Transform3d CAMERA_TRANSFORM3D      = new Transform3d(
            new Translation3d(CAMERA_Y_OFFSET*TrcUtil.METERS_PER_INCH, -CAMERA_X_OFFSET*TrcUtil.METERS_PER_INCH,
                              CAMERA_HEIGHT*TrcUtil.METERS_PER_INCH),
            new Rotation3d(0.0, Math.toRadians(-CAMERA_PITCH), Math.toRadians(-CAMERA_YAW)));
        // Camera: Logitech C310 (not used)
        public static final double WEBCAM_FX                    = 821.993;  // in pixels
        public static final double WEBCAM_FY                    = 821.993;  // in pixels
        public static final double WEBCAM_CX                    = 330.489;  // in pixels
        public static final double WEBCAM_CY                    = 248.997;  // in pixels

        public static final double CAMERA_DATA_TIMEOUT          = 0.5;      // 500ms
        public static final double VISION_TARGET_HEIGHT         = 104.0;    // Inches from the floor (not used)
        public static final double APRILTAG_SIZE                = 6.0 / TrcUtil.INCHES_PER_METER;   //  in meters
        // Homography measurements.
        // Camera rect in inches.
        public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X  = 0.0;
        public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y  = 120.0;
        public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X = CAMERA_IMAGE_WIDTH - 1;
        public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y = 120.0;
        public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X = 0.0;
        public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y = CAMERA_IMAGE_HEIGHT - 1;
        public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X = CAMERA_IMAGE_WIDTH - 1;
        public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y = CAMERA_IMAGE_HEIGHT - 1;
        public static final TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
            HOMOGRAPHY_CAMERA_TOPLEFT_X, HOMOGRAPHY_CAMERA_TOPLEFT_Y,
            HOMOGRAPHY_CAMERA_TOPRIGHT_X, HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
            HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
            HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);
        // World rect in inches.
        public static final double HOMOGRAPHY_WORLD_TOPLEFT_X   = -12.5625;
        public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y   = 48.0 - ROBOT_LENGTH/2.0 + CAMERA_Y_OFFSET;
        public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X  = 11.4375;
        public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y  = 44.75 - ROBOT_LENGTH/2.0 + CAMERA_Y_OFFSET;
        public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X= -2.5625;
        public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y= 21.0 - ROBOT_LENGTH/2.0 + CAMERA_Y_OFFSET;
        public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X = 2.5626;
        public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y = 21.0 - ROBOT_LENGTH + CAMERA_Y_OFFSET;
        public static final TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
            HOMOGRAPHY_WORLD_TOPLEFT_X, HOMOGRAPHY_WORLD_TOPLEFT_Y,
            HOMOGRAPHY_WORLD_TOPRIGHT_X, HOMOGRAPHY_WORLD_TOPRIGHT_Y,
            HOMOGRAPHY_WORLD_BOTTOMLEFT_X, HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
            HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);
    }   //class Vision

    //
    // DriveBase subsystem.
    //
    public static final DriveMode ROBOT_DRIVE_MODE              = DriveMode.ArcadeMode;
    public static final double DRIVE_RAMP_RATE                  = 0.25;

    public static final double DRIVE_SLOW_SCALE                 = 0.5;
    public static final double TURN_SLOW_SCALE                  = 0.3;
    public static final double DRIVE_NORMAL_SCALE               = 0.75;
    public static final double TURN_NORMAL_SCALE                = 0.6;

    public static class DifferentialDriveBase
    {
        public final String[] driveMotorNames                   = {"leftDriveMotor", "rightDriveMotor"};
        public final int[] driveMotorIds                        = {CANID_LFDRIVE_MOTOR, CANID_RFDRIVE_MOTOR};
        public final boolean[] driveMotorInverted               = {false, true};

        public final double DRIVE_INCHES_PER_COUNT              = 2.2421;
        public final double DRIVE_KP                            = 0.011;
        public final double DRIVE_KI                            = 0.0;
        public final double DRIVE_KD                            = 0.0013;
        public final double DRIVE_KF                            = 0.0;
        public final double DRIVE_TOLERANCE                     = 2.0;

        public final double TURN_KP                             = 0.012;
        public final double TURN_KI                             = 0.0;
        public final double TURN_KD                             = 0.0;
        public final double TURN_KF                             = 0.0;
        public final double TURN_IZONE                          = 10.0;
        public final double TURN_TOLERANCE                      = 2.0;

        public final double ROBOT_MAX_VELOCITY                  = 177.1654; // inches per second
        public final double ROBOT_MAX_ACCELERATION              = 799.1;
        public final double ROBOT_MAX_TURN_RATE                 = 572.9578;
        public final double ROBOT_VEL_KP                        = 0.0;
        public final double ROBOT_VEL_KI                        = 0.0;
        public final double ROBOT_VEL_KD                        = 0.0;
        // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
        public final double ROBOT_VEL_KF                        = 1.0 / ROBOT_MAX_VELOCITY;

        public final double DRIVE_MAX_PID_POWER                 = 0.5;
        public final double DRIVE_MAX_PID_RAMP_RATE             = 0.5;  // percentPower per sec

        public final double TURN_MAX_PID_POWER                  = 1.0;
        public final double TURN_MAX_PID_RAMP_RATE              = 1.0;  // percentPower per sec

        public final double PPD_FOLLOWING_DISTANCE              = 12.0;
        public final double PPD_POS_TOLERANCE                   = 1.0;
        public final double PPD_TURN_TOLERANCE                  = 2.0;
        public final double PPD_MOVE_DEF_OUTPUT_LIMIT           = 0.5;
        public final double PPD_ROT_DEF_OUTPUT_LIMIT            = 0.5;
    }   //class DifferentialDriveBase

    public static class MecanumDriveBase
    {
        // Drive motors.
        public final String[] driveMotorNames                   =
            {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
        public final int[] driveMotorIds                        =
            {CANID_LFDRIVE_MOTOR, CANID_RFDRIVE_MOTOR, CANID_LBDRIVE_MOTOR, CANID_RBDRIVE_MOTOR};
        public final boolean[] driveMotorInverted               = {false, true, false, true};

        // Mecanum Drive Base (not used).
        public final double DRIVE_X_INCHES_PER_COUNT            = 2.2421;
        public final double DRIVE_X_KP                          = 0.011;
        public final double DRIVE_X_KI                          = 0.0;
        public final double DRIVE_X_KD                          = 0.0013;
        public final double DRIVE_X_KF                          = 0.0;
        public final double DRIVE_X_TOLERANCE                   = 2.0;

        public final double DRIVE_Y_INCHES_PER_COUNT            = 2.2421;
        public final double DRIVE_Y_KP                          = 0.011;
        public final double DRIVE_Y_KI                          = 0.0;
        public final double DRIVE_Y_KD                          = 0.0013;
        public final double DRIVE_Y_KF                          = 0.0;
        public final double DRIVE_Y_TOLERANCE                   = 2.0;

        public final double TURN_KP                             = 0.012;
        public final double TURN_KI                             = 0.0;
        public final double TURN_KD                             = 0.0;
        public final double TURN_KF                             = 0.0;
        public final double TURN_IZONE                          = 10.0;
        public final double TURN_TOLERANCE                      = 2.0;

        public final double ROBOT_MAX_VELOCITY                  = 177.1654; // inches per second
        public final double ROBOT_MAX_ACCELERATION              = 799.1;
        public final double ROBOT_MAX_TURN_RATE                 = 572.9578;
        public final double ROBOT_VEL_KP                        = 0.0;
        public final double ROBOT_VEL_KI                        = 0.0;
        public final double ROBOT_VEL_KD                        = 0.0;
        // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
        public final double ROBOT_VEL_KF                        = 1.0 / ROBOT_MAX_VELOCITY;

        public final double DRIVE_MAX_XPID_POWER                = 0.5;
        public final double DRIVE_MAX_XPID_RAMP_RATE            = 0.5;  // percentPower per sec

        public final double DRIVE_MAX_YPID_POWER                = 0.6;
        public final double DRIVE_MAX_YPID_RAMP_RATE            = 0.5;  // percentPower per sec

        public final double DRIVE_MAX_TURNPID_POWER             = 1.0;
        public final double DRIVE_MAX_TURNPID_RAMP_RATE         = 1.0;  // percentPower per sec

        public final double PPD_FOLLOWING_DISTANCE              = 12.0;
        public final double PPD_POS_TOLERANCE                   = 1.0;
        public final double PPD_TURN_TOLERANCE                  = 2.0;
        public final double PPD_MOVE_DEF_OUTPUT_LIMIT           = 0.5;
        public final double PPD_ROT_DEF_OUTPUT_LIMIT            = 0.5;
    }   //class MecanumDriveBase

    public static class SwerveDriveBase
    {
        // Drive motors.
        public final String[] driveMotorNames                   =
            {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
        public final int[] driveMotorIds                        =
            {CANID_LFDRIVE_MOTOR, CANID_RFDRIVE_MOTOR, CANID_LBDRIVE_MOTOR, CANID_RBDRIVE_MOTOR};
        public final boolean[] driveMotorInverted               = {true, true, true, true};

        // Steer motors.
        public final String[] steerMotorNames                   =
            {"lfSteerMotor", "rfSteerMotor", "lbSteerMotor", "rbSteerMotor"};
        public final int[] steerMotorIds                        =
            {CANID_LFSTEER_MOTOR, CANID_RFSTEER_MOTOR, CANID_LBSTEER_MOTOR, CANID_RBSTEER_MOTOR};
        public final boolean[] steerMotorInverted               = {false, false, false, false};

        // Steer encoders.
        public SteerEncoderType steerEncoderType                = SteerEncoderType.Canandcoder;
        public final String[] steerEncoderNames                 =
            {"lfSteerEncoder", "rfSteerEncoder", "lbSteerEncoder", "rbSteerEncoder"};
        public int[] steerEncoderCanIds                         =
            {CANID_LFSTEER_ENCODER, CANID_RFSTEER_ENCODER, CANID_LBSTEER_ENCODER, CANID_RBSTEER_ENCODER};
        public final int[] steerEncoderAnalogIds                =
            {AIN_LFSTEER_ENCODER, AIN_RFSTEER_ENCODER, AIN_LBSTEER_ENCODER, AIN_RBSTEER_ENCODER};
        public final boolean[] steerEncoderInverted             = {false, false, false, false};

        // Swerve modules.
        public final String[] swerveModuleNames                 = {"lfWheel", "rfWheel", "lbWheel", "rbWheel"};

        // public final double SWERVE_DRIVE_INCHES_PER_COUNT       = 9.072106867127145344367826764411e-4;
        public final double DRIVE_KP                            = 0.001;    // BaseFalconSwerve: 0.12
        public final double DRIVE_KI                            = 0.0;
        public final double DRIVE_KD                            = 0.0;
        public final double DRIVE_KF                            = 0.11;     // BaseFalconSwerve: 0.0
        public final double DRIVE_IZONE                         = 5.0;
        public final double DRIVE_TOLERANCE                     = 2.0;
        public final PidCoefficients driveCoeffs                =
            new PidCoefficients(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF);
        // Drive Motor Characterization Values From SYSID
        public final double DRIVE_KS                            = 0.32; //TODO: This must be tuned to specific robot
        public final double DRIVE_KV                            = 1.51;
        public final double DRIVE_KA                            = 0.27;

        public final double TURN_KP                             = 0.012;
        public final double TURN_KI                             = 0.0;
        public final double TURN_KD                             = 0.0;
        public final double TURN_KF                             = 0.0;
        public final double TURN_IZONE                          = 10.0;
        public final double TURN_TOLERANCE                      = 2.0;

        // Not tuned (not used).
        public final double X_TIPPING_KP                        = 0.01;
        public final double X_TIPPING_KI                        = 0.0;
        public final double X_TIPPING_KD                        = 0.0;
        public final double X_TIPPING_TOLERANCE                 = 10.0;
        public final double X_TIPPING_SETTLING_TIME             = 0.2;

        public final double Y_TIPPING_KP                        = 0.01;
        public final double Y_TIPPING_KI                        = 0.0;
        public final double Y_TIPPING_KD                        = 0.0;
        public final double Y_TIPPING_TOLERANCE                 = 10.0;
        public final double Y_TIPPING_SETTLING_TIME             = 0.2;

        public final double ROBOT_MAX_VELOCITY                  = 177.1654; // inches per second
        public final double ROBOT_MAX_ACCELERATION              = 799.1;
        public final double ROBOT_MAX_TURN_RATE                 = 572.9578;
        public final double ROBOT_VEL_KP                        = 0.0;
        public final double ROBOT_VEL_KI                        = 0.0;
        public final double ROBOT_VEL_KD                        = 0.0;
        // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
        public final double ROBOT_VEL_KF                        = 1.0 / ROBOT_MAX_VELOCITY;

        public final double DRIVE_MAX_XPID_POWER                = 0.5;
        public final double DRIVE_MAX_XPID_RAMP_RATE            = 0.5;  // percentPower per sec

        public final double DRIVE_MAX_YPID_POWER                = 0.6;
        public final double DRIVE_MAX_YPID_RAMP_RATE            = 0.5;  // percentPower per sec

        public final double DRIVE_MAX_TURNPID_POWER             = 1.0;
        public final double DRIVE_MAX_TURNPID_RAMP_RATE         = 1.0;  // percentPower per sec

        // Applicable only for Swerve Drive.
        public final double CANCODER_CPR                        = 4096.0;
        public final double FALCON_CPR                          = 2048.0;
        public final double FALCON_MAX_RPM                      = 6380.0;

        public final double DRIVE_GEAR_RATIO                    = 9.63;
        public final double DRIVE_WHEEL_CIRCUMFERENCE           = 4.0 * Math.PI;
        public final double DRIVE_INCHES_PER_COUNT              = DRIVE_WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;

        public double STEER_GEAR_RATIO                          = 15.43;
        public final double STEER_DEGREES_PER_COUNT             = 360.0 / STEER_GEAR_RATIO;
        // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
        public final double STEER_MAX_VEL                       = (FALCON_MAX_RPM*0.81/STEER_GEAR_RATIO/60.0)*360.0;

        // Zeroes are normalized offsets which are in the unit of percentage revolution (0.0 to 1.0).
        // This is a backup if file is not found: LF, RF, LB, RB.
        public final double[] STEER_ZEROS                       = new double[] {0.0, 0.0, 0.0, 0.0};

        // public final PidCoefficients magicSteerCoeff            =
        //     new PidCoefficients(2.0, 0.01, 0.0, 1023.0 / STEER_MAX_VEL_COUNT_PER_100MS, 5.0 / STEER_DEGREES_PER_COUNT);
        public final double STEER_KP                            = 3.0;
        public final double STEER_KI                            = 0.0;
        public final double STEER_KD                            = 0.0;
        // kF set to Motion Magic recommendation.
        public final double STEER_KF                            = 0.0;//1023.0 / STEER_MAX_VEL_COUNT_PER_100MS;
        // iZone set to within 5 steering degrees.
        public final double STEER_IZONE                         = 0.0;//5.0 / STEER_DEGREES_PER_COUNT;
        public final PidCoefficients steerCoeffs                =
            new PidCoefficients(STEER_KP, STEER_KI, STEER_KD, STEER_KF, STEER_IZONE);

        public final double PPD_FOLLOWING_DISTANCE              = 12.0;
        public final double PPD_POS_TOLERANCE                   = 1.0;
        public final double PPD_TURN_TOLERANCE                  = 2.0;
        public final double PPD_MOVE_DEF_OUTPUT_LIMIT           = 0.5;
        public final double PPD_ROT_DEF_OUTPUT_LIMIT            = 0.5;

        //
        // Command-based constatns.
        //
        public final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-
        // Drivetrain Constants
        public final double trackWidth = Units.inchesToMeters(ROBOT_WHEELBASE_WIDTH);
        public final double wheelBase = Units.inchesToMeters(ROBOT_WHEELBASE_LENGTH);
        public final double wheelCircumference = Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFERENCE);
        // Swerve Kinematics
        // No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
        public final SwerveDriveKinematics swerveKinematics  = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        // Meters per Second
        public final double maxSpeed = Units.inchesToMeters(ROBOT_MAX_VELOCITY);
        // Radians per Second
        public final double maxAngularVelocity = Units.degreesToRadians(ROBOT_MAX_TURN_RATE);
    }   //class SwerveDriveBase

    public static final class AutoConstants
    {
        //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 4.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.0;//0.00149
        public static final double kPYController = 0.0;
        public static final double kPThetaController = 0.0;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    //
    // Other subsystems.
    //

    public static class Intake
    {
        public static final int candId                          = CANID_INTAKE_MOTOR;
        public static final boolean motorInverted               = false;    //TODO: tune
        public static final TrcPidConveyor.Parameters params    = new TrcPidConveyor.Parameters()
            .setMaxCapacity(1)
            .setMovePower(1.0)                              //TODO: tune
            .setObjectDistance(16.0);
        public static final int entrySensorChannel              = DIO_INTAKE_ENTRY;
        public static final int exitSensorChannel               = DIO_INTAKE_EXIT;
        public static final boolean entrySensorInverted         = false;    //TODO: tune
        public static final boolean exitSensorInverted          = false;    //TODO: tune
        public static final double posScale                     = 1.0;      //TODO: tune
        public static final PidCoefficients posPidCoeff         = new PidCoefficients(0.0, 0.0, 0.0);   //TODO: tune
    }   //class Intake

    public static class Shooter
    {
        public static final int shooterCandId                   = CANID_SHOOTER_MOTOR;
        public static final boolean shooterMotorInverted        = false;    //TODO: tune
        public static final double shooterGearRatio             = 1.0;      //TODO: tune
        public static final double shooterPosScale              = 1.0 / shooterGearRatio;   // in Revolutions
        public static final PidCoefficients shooterVelPidCoeff  = new PidCoefficients(0.0, 0.0, 0.0, 0.0);  //TODO: tune

        public static final int tilterCanId                     = CANID_TILTER_MOTOR;
        public static final boolean tilterMotorInverted         = false;    //TODO: tune
        public static final double tilterGearRatio              = 1.0;      //TODO: tune
        public static final double tilterPosScale               = 360.0 / tilterGearRatio;  // in Degrees
        public static final double tilterPosOffset              = 15.0;     //TODO: tune, in degrees
        public static final double tilterPowerLimit             = 0.5;      //TODO: tune
        public static final PidCoefficients tilterPosPidCoeff   = new PidCoefficients(0.0, 0.0, 0.0, 0.0);  //TODO: tune
    }   //class Shooter

    public static class Climber
    {
        public static final int climberCandId                   = CANID_CLIMBER_MOTOR;
        public static final boolean motorInverted               = false;    //TODO: tune
        public static final double climberPosScale                     = 1.0;      //TODO: tune
        public static final PidCoefficients climberPosPidCoeff         = new PidCoefficients(0.0, 0.0, 0.0);   //TODO: tune
    }   //class Climber

    public static class Deployer
    {

    }   //class Deployer

}   //class RobotParams
