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
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcPidController.PidCoefficients;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import team492.autotasks.ShootParamTable;
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

    public static class Preferences
    {
        // Global config
        public static final RobotType robotType                 = RobotType.SwerveRobot;
        public static final boolean inCompetition               = false;
        public static final boolean hybridMode                  = false;
        public static final boolean useTraceLog                 = true;
        // Status Update
        public static final boolean doStatusUpdate              = !inCompetition;
        public static final boolean showLoopTime                = false;
        public static final boolean showPowerConsumption        = false;
        public static final boolean showDriveBase               = false;
        public static final boolean showPurePursuitDrive        = false;
        public static final boolean showPidDrive                = false;
        public static final boolean showVision                  = true;
        public static final boolean showSubsystems              = false;
        // Inputs
        public static final boolean useDriverXboxController     = true;
        public static final boolean useOperatorXboxController   = true;
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
        public static final boolean useAntiTipping              = false;
        // Subsystems
        public static final boolean useSubsystems               = true;
        public static final boolean useIntake                   = true;
        public static final boolean useShooter                  = true;
        public static final boolean useClimber                  = true;
    }   //class Preferences

    public static final String TEAM_FOLDER_PATH                 = "/home/lvuser/trc492";
    public static final double DASHBOARD_UPDATE_INTERVAL        = 0.1;      // in msec

    public static class Field
    {
        // Field dimensions in inches.
        public static final double LENGTH                       = 651.2;    //54.0*12.0;
        public static final double WIDTH                        = 323.28;   //27.0*12.0;
    }   //class Field

    public static class Robot
    {
        public static final String NAME                         = "Robot492";
        // Robot dimensions in inches.
        public static final double WIDTH                        = 34.5;     // Frame dimensions, including bumpers.
        public static final double LENGTH                       = 34.5;     // Frame dimensions, including bumpers.

        public static final double WHEELBASE_WIDTH              = 23.25;    // Required by swerve drive base.
        public static final double WHEELBASE_LENGTH             = 23.25;    // Required by swerve drive base.
    }   //class Robot

    public static class Game
    {
        //
        // Robot starting positions.
        //
        public static final double SW_ANGLE_RADIAN              = Math.toRadians(63.5);

        public static final TrcPose2D STARTPOS_BLUE_SW_SOURCE_SIDE= new TrcPose2D(
            -(Field.WIDTH / 2.0 + 57.0) +
            (20.25 + 20.25 * Math.cos(SW_ANGLE_RADIAN) + Robot.LENGTH / 2.0 * Math.sin(SW_ANGLE_RADIAN)),
            20.25 * Math.sin(SW_ANGLE_RADIAN) + (Robot.LENGTH / 2.0) * Math.cos(SW_ANGLE_RADIAN), 180.0 + 63.5);
        public static final TrcPose2D STARTPOS_BLUE_SW_CENTER   = new TrcPose2D(
            -(Field.WIDTH / 2.0 + 57.0), 34.17 + Robot.LENGTH / 2.0, 180.0);
        public static final TrcPose2D STARTPOS_BLUE_SW_AMP_SIDE = new TrcPose2D(
            -(Field.WIDTH / 2.0 + 57.0) -
            (20.25 + 20.25 * Math.cos(SW_ANGLE_RADIAN) + Robot.LENGTH / 2.0 * Math.sin(SW_ANGLE_RADIAN)),
            20.25 * Math.sin(SW_ANGLE_RADIAN) + (Robot.LENGTH / 2.0) * Math.cos(SW_ANGLE_RADIAN), 180.0 - 63.5);
        public static final TrcPose2D STARTPOS_BLUE_AMP         = new TrcPose2D(
            -Field.WIDTH + 17.75 + Robot.LENGTH / 2.0, 74.1 - Robot.WIDTH / 2.0, -90.0);

        public static final TrcPose2D STARTPOS_RED_SW_SOURCE_SIDE= new TrcPose2D(
            -(Field.WIDTH / 2.0 + 57.0) +
            (20.25 + 20.25 * Math.cos(SW_ANGLE_RADIAN) + Robot.LENGTH / 2.0 * Math.sin(SW_ANGLE_RADIAN)),
            Field.LENGTH - (20.25 * Math.sin(SW_ANGLE_RADIAN) + (Robot.LENGTH / 2.0) * Math.cos(SW_ANGLE_RADIAN)),
            -63.5);
        public static final TrcPose2D STARTPOS_RED_SW_CENTER    = new TrcPose2D(
            -(Field.WIDTH / 2.0 + 57.0), Field.LENGTH - (34.7 + Robot.LENGTH / 2.0), 0.0);
        public static final TrcPose2D STARTPOS_RED_SW_AMP_SIDE  = new TrcPose2D(
            -(Field.WIDTH / 2.0 + 57.0) -
            (20.25 + 20.25 * Math.cos(SW_ANGLE_RADIAN) + Robot.LENGTH / 2.0 * Math.sin(SW_ANGLE_RADIAN)),
            Field.LENGTH - (20.25 * Math.sin(SW_ANGLE_RADIAN) + (Robot.LENGTH / 2.0) * Math.cos(SW_ANGLE_RADIAN)),
            63.5);
        public static final TrcPose2D STARTPOS_RED_AMP          = new TrcPose2D(
            -Field.WIDTH + 17.75 + Robot.LENGTH / 2.0, Field.LENGTH - (74.1 - Robot.WIDTH / 2.0), -90.0);

        public static final TrcPose2D[][] startPoses =
        {
            {STARTPOS_BLUE_SW_SOURCE_SIDE, STARTPOS_BLUE_SW_CENTER, STARTPOS_BLUE_SW_AMP_SIDE, STARTPOS_BLUE_AMP},
            {STARTPOS_RED_SW_SOURCE_SIDE, STARTPOS_RED_SW_CENTER, STARTPOS_RED_SW_AMP_SIDE, STARTPOS_RED_AMP}
        };
        //
        // Game element locations and dimensions.
        //
        public static final TrcPose2D WINGNOTE_BLUE_SOURCE_SIDE = new TrcPose2D(
            -Field.WIDTH / 2.0, 114.0, 0.0);
        public static final TrcPose2D WINGNOTE_BLUE_SW_SIDE     = new TrcPose2D(
            -Field.WIDTH / 2.0 - 57.0 * 1, 114.0, 0.0);
        public static final TrcPose2D WINGNOTE_BLUE_AMP_SIDE    = new TrcPose2D(
            -Field.WIDTH / 2.0 - 57.0 * 2, 114.0, 0.0);

        public static final TrcPose2D WINGNOTE_RED_SOURCE_SIDE  = new TrcPose2D(
            -Field.WIDTH / 2.0, Field.LENGTH - 114.0, 0.0);
        public static final TrcPose2D WINGNOTE_RED_SW_SIDE      = new TrcPose2D(
            -Field.WIDTH / 2.0 - 57.0 * 1, Field.LENGTH - 114.0, 0.0);
        public static final TrcPose2D WINGNOTE_RED_AMP_SIDE     = new TrcPose2D(
            -Field.WIDTH / 2.0 - 57.0 * 2, Field.LENGTH - 114.0, 0.0);

        public static final TrcPose2D[][] wingNotePoses =
        {
            {WINGNOTE_BLUE_SOURCE_SIDE,  WINGNOTE_BLUE_SW_SIDE, WINGNOTE_BLUE_AMP_SIDE},
            {WINGNOTE_RED_SOURCE_SIDE,  WINGNOTE_RED_SW_SIDE, WINGNOTE_RED_AMP_SIDE}
        };

        public static final TrcPose2D CENTERLINE_NOTE_1         = new TrcPose2D(
            -29.64, Field.LENGTH / 2.0, 0.0);
        public static final TrcPose2D CENTERLINE_NOTE_2         = new TrcPose2D(
            -29.64 - 66.0 * 1, Field.LENGTH / 2.0, 0);
        public static final TrcPose2D CENTERLINE_NOTE_3         = new TrcPose2D(
            -29.64 - 66.0 * 2, Field.LENGTH / 2.0, 0);
        public static final TrcPose2D CENTERLINE_NOTE_4         = new TrcPose2D(
            -29.64 - 66.0 * 3, Field.LENGTH / 2.0, 0);
        public static final TrcPose2D CENTERLINE_NOTE_5         = new TrcPose2D(
            -29.64 - 66.0 * 4, Field.LENGTH / 2.0, 0);

        public static final TrcPose2D[] centerlineNotePoses =
        {
            CENTERLINE_NOTE_1, CENTERLINE_NOTE_2, CENTERLINE_NOTE_3, CENTERLINE_NOTE_4, CENTERLINE_NOTE_5
        };
    }   //class Game

    public static class HWConfig
    {
        //
        // Joystick ports.
        //
        public static final int XBOX_DRIVER_CONTROLLER          = 0;
        public static final int JSPORT_DRIVER_LEFTSTICK         = 0;
        public static final int JSPORT_DRIVER_RIGHTSTICK        = 1;
        public static final int XBOX_OPERATOR_CONTROLLER        = 2;
        public static final int JSPORT_OPERATORSTICK            = 2;
        public static final int JSPORT_BUTTON_PANEL             = 3;
        public static final int JSPORT_SWITCH_PANEL             = 4;
        //
        // CAN IDs.
        //
        public static final int CANID_LFDRIVE_MOTOR             = 3;
        public static final int CANID_RFDRIVE_MOTOR             = 4;
        public static final int CANID_LBDRIVE_MOTOR             = 5;
        public static final int CANID_RBDRIVE_MOTOR             = 6;

        // Applicable only for Swerve Drive.
        public static final int CANID_LFSTEER_MOTOR             = 13;
        public static final int CANID_RFSTEER_MOTOR             = 14;
        public static final int CANID_LBSTEER_MOTOR             = 15;
        public static final int CANID_RBSTEER_MOTOR             = 16;

        // Applicable only for Swerve Drive.
        public static final int CANID_LFSTEER_ENCODER           = 23;
        public static final int CANID_RFSTEER_ENCODER           = 24;
        public static final int CANID_LBSTEER_ENCODER           = 25;
        public static final int CANID_RBSTEER_ENCODER           = 26;

        // Subsystems.
        public static final int CANID_TILT_MOTOR                = 7;
        public static final int CANID_INTAKE_MOTOR              = 8;
        public static final int CANID_CLIMBER_MOTOR             = 9;
        public static final int CANID_SHOOTER_MOTOR             = 17;

        public static final int CANID_PCM                       = 30;
        public static final int CANID_PDP                       = 31;
        //
        // PDP Channels.
        //
        public static final int PDP_CHANNEL_LFDRIVE_MOTOR       = 11;   // TODO: Need updating
        public static final int PDP_CHANNEL_RFDRIVE_MOTOR       = 5;
        public static final int PDP_CHANNEL_LBDRIVE_MOTOR       = 13;
        public static final int PDP_CHANNEL_RBDRIVE_MOTOR       = 3;
        public static final int PDP_CHANNEL_LFSTEER_MOTOR       = 10;
        public static final int PDP_CHANNEL_RFSTEER_MOTOR       = 6;
        public static final int PDP_CHANNEL_LBSTEER_MOTOR       = 12;
        public static final int PDP_CHANNEL_RBSTEER_MOTOR       = 4;

        public static final int PDP_CHANNEL_ROBORIO             = 20;
        public static final int PDP_CHANNEL_VRM                 = 18;
        public static final int PDP_CHANNEL_PCM                 = 19;
        public static final int PDP_CHANNEL_RADIO_POE           = 22;
        public static final int PDP_CHANNEL_ETHERNET_SWITCH     = 21;
        public static final int PDP_CHANNEL_LIMELIGHT           = 0;
        public static final int PDP_CHANNEL_LED                 = 14;

        public static final double BATTERY_CAPACITY_WATT_HOUR   = 18.0*12.0;
        //
        // Analog Input ports (not used).
        //
        public static final int AIN_PRESSURE_SENSOR             = 0;
        public static final int AIN_LFSTEER_ENCODER             = 0;
        public static final int AIN_RFSTEER_ENCODER             = 1;
        public static final int AIN_LBSTEER_ENCODER             = 2;
        public static final int AIN_RBSTEER_ENCODER             = 3;
        //
        // Digital Input/Output ports.
        //
        public static final int DIO_INTAKE_ENTRY                = 0;
        public static final int DIO_INTAKE_EXIT                 = 1;

        //
        // PWM channels.
        //
        public static final int NUM_LEDS                        = 30;
        public static final int PWM_CHANNEL_LED                 = 9;
        //
        // Relay channels.
        //

        //
        // Pneumatic channels.
        //

        //
        // Ultrasonic sensors.
        //
        // public static final double SONAR_INCHES_PER_VOLT        = 1.0/0.0098; //9.8mV per inch
        // public static final double SONAR_ERROR_THRESHOLD        = 50.0; //value should not jump 50-in per time slice.
    }   //class HWConfig
    //
    // Vision subsystem.
    //
    public static class Vision
    {
        public static final int FRONTCAM_IMAGE_WIDTH            = 1280;     // in pixels
        public static final int FRONTCAM_IMAGE_HEIGHT           = 800;      // in pixels
        // Camera location on robot.
        public static final double FRONTCAM_X_OFFSET            = -3.5;   // Inches to the right from robot center
        public static final double FRONTCAM_Y_OFFSET            = -2.375;     // Inches forward from robot center
        public static final double FRONTCAM_Z_OFFSET            = 23.125;     // Inches up from the floor
        public static final double FRONTCAM_PITCH               = 33.0;     // degrees up from horizontal
        public static final double FRONTCAM_YAW                 = 0.0;      // degrees clockwise from robot front
        public static final double FRONTCAM_ROLL                = 0.0;
        public static final Transform3d robotToFrontCam         = new Transform3d(
            new Translation3d(Units.inchesToMeters(FRONTCAM_Y_OFFSET), -Units.inchesToMeters(FRONTCAM_X_OFFSET),
                              Units.inchesToMeters(FRONTCAM_Z_OFFSET)),
            new Rotation3d(Units.degreesToRadians(FRONTCAM_ROLL), Units.degreesToRadians(-FRONTCAM_PITCH),
                           Units.degreesToRadians(-FRONTCAM_YAW)));
        public static final TrcPose2D robotToFrontCamPose       = new TrcPose2D(
            FRONTCAM_X_OFFSET, FRONTCAM_Y_OFFSET, FRONTCAM_YAW);

        public static final int BACKCAM_IMAGE_WIDTH             = 1280;     // in pixels
        public static final int BACKCAM_IMAGE_HEIGHT            = 800;      // in pixels
        // Camera location on robot.
        public static final double BACKCAM_X_OFFSET             = -0.5;      // Inches to the right from robot center
        public static final double BACKCAM_Y_OFFSET             = -5.375;    // Inches forward from robot center
        public static final double BACKCAM_Z_OFFSET             = 20.0;    // Inches up from the floor
        public static final double BACKCAM_PITCH                = -17.5;    // degrees up from horizontal
        public static final double BACKCAM_YAW                  = 180.0;    // degrees clockwise from robot front
        public static final double BACKCAM_ROLL                 = 0.0;
        public static final Transform3d robotToBackCam          = new Transform3d(
            new Translation3d(Units.inchesToMeters(BACKCAM_Y_OFFSET), -Units.inchesToMeters(BACKCAM_X_OFFSET),
                              Units.inchesToMeters(BACKCAM_Z_OFFSET)),
            new Rotation3d(Units.degreesToRadians(BACKCAM_ROLL), Units.degreesToRadians(-BACKCAM_PITCH),
            Units.degreesToRadians(-BACKCAM_YAW)));
        public static final TrcPose2D robotToBackCamPose        = new TrcPose2D(
            BACKCAM_X_OFFSET, BACKCAM_Y_OFFSET, BACKCAM_YAW);
        // Camera: Logitech C310 (not used)
        public static final double WEBCAM_FX                    = 821.993;  // in pixels
        public static final double WEBCAM_FY                    = 821.993;  // in pixels
        public static final double WEBCAM_CX                    = 330.489;  // in pixels
        public static final double WEBCAM_CY                    = 248.997;  // in pixels

        public static final double CAMERA_DATA_TIMEOUT          = 0.5;      // 500ms
        public static final double VISION_TARGET_HEIGHT         = 104.0;    // Inches from the floor (not used)
        public static final double APRILTAG_SIZE                = Units.inchesToMeters(6.5);    //  in meters
        // Homography measurements (not used).
        // Camera rect in inches.
        public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X  = 0;
        public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y  = 400.0;
        public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X = BACKCAM_IMAGE_WIDTH - 1;
        public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y = 400.0;
        public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X = 0.0;
        public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y = BACKCAM_IMAGE_HEIGHT - 1;
        public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X = BACKCAM_IMAGE_WIDTH - 1;
        public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y = BACKCAM_IMAGE_HEIGHT - 1;
        public static final TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
            HOMOGRAPHY_CAMERA_TOPLEFT_X, HOMOGRAPHY_CAMERA_TOPLEFT_Y,
            HOMOGRAPHY_CAMERA_TOPRIGHT_X, HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
            HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
            HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);
        // World rect in inches.
        public static final double HOMOGRAPHY_WORLD_TOPLEFT_X   = -46.0;
        public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y   =  76.5 - Robot.LENGTH/2.0 + BACKCAM_Y_OFFSET;
        public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X  = 33.0;
        public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y  = 73.5 - Robot.LENGTH/2.0 + BACKCAM_Y_OFFSET;
        public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X= -17.0;
        public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y= 42.5 - Robot.LENGTH/2.0 + BACKCAM_Y_OFFSET;
        public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X = 15.0;
        public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y = 41.0 - Robot.LENGTH/2.0 + BACKCAM_Y_OFFSET;
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

    public static final double DRIVE_SLOW_SCALE                 = 0.3;
    public static final double TURN_SLOW_SCALE                  = 0.3;
    public static final double DRIVE_NORMAL_SCALE               = 0.8;
    public static final double TURN_NORMAL_SCALE                = 0.6;

    public static class SwerveDriveBase
    {
        public static final String STEER_ZERO_CAL_FILE          = "SteerCalibration.txt";
        // Drive motors.
        public final String[] driveMotorNames                   =
            {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
        public final int[] driveMotorIds                        =
            {HWConfig.CANID_LFDRIVE_MOTOR, HWConfig.CANID_RFDRIVE_MOTOR,
             HWConfig.CANID_LBDRIVE_MOTOR, HWConfig.CANID_RBDRIVE_MOTOR};
        public final boolean[] driveMotorInverted               = {false, false, false, false};

        // Steer motors.
        public final String[] steerMotorNames                   =
            {"lfSteerMotor", "rfSteerMotor", "lbSteerMotor", "rbSteerMotor"};
        public final int[] steerMotorIds                        =
            {HWConfig.CANID_LFSTEER_MOTOR, HWConfig.CANID_RFSTEER_MOTOR,
             HWConfig.CANID_LBSTEER_MOTOR, HWConfig.CANID_RBSTEER_MOTOR};
        public final boolean[] steerMotorInverted               = {false, false, false, false};

        // Steer encoders.
        public SteerEncoderType steerEncoderType                = SteerEncoderType.Canandcoder;
        public final String[] steerEncoderNames                 =
            {"lfSteerEncoder", "rfSteerEncoder", "lbSteerEncoder", "rbSteerEncoder"};
        public int[] steerEncoderCanIds                         =
            {HWConfig.CANID_LFSTEER_ENCODER, HWConfig.CANID_RFSTEER_ENCODER,
             HWConfig.CANID_LBSTEER_ENCODER, HWConfig.CANID_RBSTEER_ENCODER};
        public final int[] steerEncoderAnalogIds                =
            {HWConfig.AIN_LFSTEER_ENCODER, HWConfig.AIN_RFSTEER_ENCODER,
             HWConfig.AIN_LBSTEER_ENCODER, HWConfig.AIN_RBSTEER_ENCODER};
        public final boolean[] steerEncoderInverted             = {false, false, false, false};

        // Swerve modules.
        public final String[] swerveModuleNames                 = {"lfWheel", "rfWheel", "lbWheel", "rbWheel"};

        // public final double SWERVE_DRIVE_INCHES_PER_COUNT       = 9.072106867127145344367826764411e-4;
        public final double DRIVE_KP                            = 0.017;    // BaseFalconSwerve: 0.12
        public final double DRIVE_KI                            = 0.0;
        public final double DRIVE_KD                            = 0.0025;
        public final double DRIVE_KF                            = 0.0;      // BaseFalconSwerve: 0.11
        public final double DRIVE_IZONE                         = 5.0;
        public final double DRIVE_TOLERANCE                     = 1.0;
        // Drive Motor Characterization Values From SYSID
        public final double DRIVE_KS                            = 0.32;     //TODO: This must be tuned to specific robot
        public final double DRIVE_KV                            = 1.51;
        public final double DRIVE_KA                            = 0.27;

        public final double TURN_KP                             = 0.0065;
        public final double TURN_KI                             = 0.0;
        public final double TURN_KD                             = 0.0004;
        public final double TURN_KF                             = 0.0;
        public final double TURN_IZONE                          = 10.0;
        public final double TURN_TOLERANCE                      = 1.0;

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

        public static final double ROBOT_MAX_VELOCITY           = 171.0;    // inches per second
        public static final double ROBOT_MAX_ACCELERATION       = 23000.0;  // inches per second square
        public static final double ROBOT_MAX_TURN_RATE          = 1450.0;   // degrees per second
        public static final double PROFILED_MAX_VELOCITY        = 157.48;   // inches per second
        public static final double PROFILED_MAX_ACCELERATION    = 157.48;   // inches per second square
        public static final double PROFILED_MAX_TURN_RATE       = 180.0;    // degrees per second
        public final double ROBOT_VEL_KP                        = 0.0;
        public final double ROBOT_VEL_KI                        = 0.0;
        public final double ROBOT_VEL_KD                        = 0.0;
        // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance),
        // units: sec./in.
        public final double ROBOT_VEL_KF                        = 1.0 / PROFILED_MAX_VELOCITY;

        public final double DRIVE_MAX_XPID_POWER                = 0.5;
        public final double DRIVE_MAX_XPID_RAMP_RATE            = 0.5;      // percentPower per sec

        public final double DRIVE_MAX_YPID_POWER                = 0.6;
        public final double DRIVE_MAX_YPID_RAMP_RATE            = 0.5;      // percentPower per sec

        public final double DRIVE_MAX_TURNPID_POWER             = 1.0;
        public final double DRIVE_MAX_TURNPID_RAMP_RATE         = 1.0;      // percentPower per sec

        // Applicable only for Swerve Drive.
        public final double CANCODER_CPR                        = 4096.0;
        public final double FALCON_CPR                          = 2048.0;
        public final double FALCON_MAX_RPM                      = 6380.0;

        // Tuned 02/29/2024: WheelDiameter = 3.9326556997620689090425924610785
        public double DRIVE_GEAR_RATIO                          = 6.75;
        public final double DRIVE_WHEEL_DIAMETER                = 3.9326556997620689090425924610785;
        public final double DRIVE_INCHES_PER_ROT                = DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_GEAR_RATIO;

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
        public final double STEER_KF                            = 0.0;
        // iZone set to within 5 steering degrees.
        public final double STEER_IZONE                         = 0.0;
        public final PidCoefficients steerPosCoeffs             =
            new PidCoefficients(STEER_KP, STEER_KI, STEER_KD, STEER_KF, STEER_IZONE);
        public final double steerPosTolerance                   = 0.5;      // in degrees.

        public final double PPD_FOLLOWING_DISTANCE              = 12.0;
        public final double PPD_POS_TOLERANCE                   = 1.0;
        public final double PPD_TURN_TOLERANCE                  = 2.0;
        public final double PPD_MOVE_DEF_OUTPUT_LIMIT           = 0.5;
        public final double PPD_ROT_DEF_OUTPUT_LIMIT            = 0.5;

        //
        // Command-based constants.
        //
        public final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-
        // Drivetrain Constants
        public final double trackWidth = Units.inchesToMeters(Robot.WHEELBASE_WIDTH);
        public final double wheelBase = Units.inchesToMeters(Robot.WHEELBASE_LENGTH);
        public final double wheelCircumference = Units.inchesToMeters(DRIVE_WHEEL_DIAMETER * Math.PI);
        // Swerve Kinematics
        // No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
        public final SwerveDriveKinematics swerveKinematics  = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        // Meters per Second
        public final double maxSpeed = Units.inchesToMeters(PROFILED_MAX_VELOCITY);
        // Radians per Second
        public final double maxAngularVelocity = Units.degreesToRadians(PROFILED_MAX_TURN_RATE);
    }   //class SwerveDriveBase

    public static class MecanumDriveBase
    {
        // Drive motors.
        public final String[] driveMotorNames                   =
            {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
        public final int[] driveMotorIds                        =
            {HWConfig.CANID_LFDRIVE_MOTOR, HWConfig.CANID_RFDRIVE_MOTOR,
             HWConfig.CANID_LBDRIVE_MOTOR, HWConfig.CANID_RBDRIVE_MOTOR};
        public final boolean[] driveMotorInverted               = {false, true, false, true};

        // Mecanum Drive Base (not used).
        public final double DRIVE_X_INCHES_PER_COUNT            = 2.2421;
        public final double DRIVE_X_KP                          = 0.011;
        public final double DRIVE_X_KI                          = 0.0;
        public final double DRIVE_X_KD                          = 0.0013;
        public final double DRIVE_X_KF                          = 0.0;
        public final double DRIVE_X_TOLERANCE                   = 1.0;

        public final double DRIVE_Y_INCHES_PER_COUNT            = 2.2421;
        public final double DRIVE_Y_KP                          = 0.011;
        public final double DRIVE_Y_KI                          = 0.0;
        public final double DRIVE_Y_KD                          = 0.0013;
        public final double DRIVE_Y_KF                          = 0.0;
        public final double DRIVE_Y_TOLERANCE                   = 1.0;

        public final double TURN_KP                             = 0.012;
        public final double TURN_KI                             = 0.0;
        public final double TURN_KD                             = 0.0;
        public final double TURN_KF                             = 0.0;
        public final double TURN_IZONE                          = 10.0;
        public final double TURN_TOLERANCE                      = 2.0;

        public static final double ROBOT_MAX_VELOCITY           = 177.1654; // inches per second
        public static final double ROBOT_MAX_ACCELERATION       = 799.1;
        public static final double ROBOT_MAX_TURN_RATE          = 572.9578;
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

    public static class DifferentialDriveBase
    {
        public final String[] driveMotorNames                   = {"leftDriveMotor", "rightDriveMotor"};
        public final int[] driveMotorIds                        =
            {HWConfig.CANID_LFDRIVE_MOTOR, HWConfig.CANID_RFDRIVE_MOTOR};
        public final boolean[] driveMotorInverted               = {false, true};

        public final double DRIVE_INCHES_PER_COUNT              = 2.2421;
        public final double DRIVE_KP                            = 0.011;
        public final double DRIVE_KI                            = 0.0;
        public final double DRIVE_KD                            = 0.0013;
        public final double DRIVE_KF                            = 0.0;
        public final double DRIVE_TOLERANCE                     = 1.0;

        public final double TURN_KP                             = 0.012;
        public final double TURN_KI                             = 0.0;
        public final double TURN_KD                             = 0.0;
        public final double TURN_KF                             = 0.0;
        public final double TURN_IZONE                          = 10.0;
        public final double TURN_TOLERANCE                      = 2.0;

        public static final double ROBOT_MAX_VELOCITY           = 177.1654; // inches per second
        public static final double ROBOT_MAX_ACCELERATION       = 799.1;
        public static final double ROBOT_MAX_TURN_RATE          = 572.9578;
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
        public static final int motorCandId                     = HWConfig.CANID_INTAKE_MOTOR;
        public static final boolean motorInverted               = false;
        public static final int entrySensorChannel              = HWConfig.DIO_INTAKE_ENTRY;
        public static final int exitSensorChannel               = HWConfig.DIO_INTAKE_EXIT;
        public static final boolean entrySensorInverted         = true;
        public static final boolean exitSensorInverted          = true;

        public static final double intakePower                  = 0.85;
        public static final double ejectForwardPower            = 0.5;
        public static final double noteDistanceThreshold        = 72.0;
    }   //class Intake

    public static class Shooter
    {
        public static final int shooterCandId                   = HWConfig.CANID_SHOOTER_MOTOR;
        public static final boolean shooterMotorInverted        = false;
        public static final double shooterGearRatio             = 1.0;
        public static final double shooterPosScale              = 1.0 / shooterGearRatio;   // in rot.
        public static final PidCoefficients shooterVelPidCoeff  = new PidCoefficients(0.38, 0.0, 0.000098, 0.120);
        public static final double shooterVelTolerance          = 3.0;      // in rps.
        public static final double shooterMaxVelocity           = 100.0;    // in rps.
        public static final double shooterMaxAcceleration       = 100.0;    // in rps square.
        public static final double shooterVelMinInc             = 1.0;      // in rps.
        public static final double shooterVelMaxInc             = 10.0;     // in rps.
        public static final double shooterSpeakerCloseVelocity  = 90.0;     // in rps.
        public static final double shooterAmpVelocity           = 20.0;     // in rps.
        public static final double shooterSourcePickupVelocity  = -20.0;    // in rps.
        public static final double shooterPresetVelTolerance    = 5.0;      // in rps.
        public static final double[] shooterPresetVelocities    = new double[]
            {20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0};

        public static final int tiltCanId                       = HWConfig.CANID_TILT_MOTOR;
        public static final boolean tiltMotorInverted           = true;
        public static final double tiltGearRatio                = 59.0/18.0;
        public static final double tiltPosScale                 = 360.0 / tiltGearRatio;
        public static final double tiltPosOffset                = -15.0;    // in degrees
        public static final double tiltZeroOffset               = 0.018;    // in raw encoder unit
        public static final double tiltPowerLimit               = 0.5;
        public static final PidCoefficients tiltPosPidCoeff     = new PidCoefficients(0.023, 0.0, 0.001, 0.0);
        public static final double tiltPosPidTolerance          = 1.0;
        public static final double tiltMinAngle                 = 0.0;
        public static final double tiltMaxAngle                 = 87.0;     // in degrees.
        public static final double tiltAngleMinInc              = 1.0;      // in degrees.
        public static final double tiltAngleMaxInc              = 10.0;     // in degrees.
        public static final double tiltTurtleAngle              = 35.0;     // in degrees.
        public static final double tiltSpeakerFarAngle          = 52.0;     // in degrees.
        public static final double tiltAmpAngle                 = 57.5;     // in degrees.
        public static final double tiltSpeakerCloseAngle        = 60.0;     // in degrees.
        public static final double tiltSourcePickupAngle        = 88.0;     // in degrees.

        public static final double tiltPresetPosTolerance       = 1.0;      // in degrees.
        public static final double[] tiltPresetPositions        = new double[]
            {tiltTurtleAngle, tiltSpeakerFarAngle, tiltAmpAngle, tiltSpeakerCloseAngle, tiltSourcePickupAngle};

        // Talked with Jackson and said that we would most likely not score in trap,
        // so I don't think there is a need to tune ... Will leave it just in case.
        // public static final ShootParamTable.Params stageShootParams = new ShootParamTable.Params(
        //     "Stage", 0.0, 30.0, 60.0);

        public static final String SPEAKER_PARAM_NAME           = "Speaker0ft";
        public static final ShootParamTable speakerShootParamTable = new ShootParamTable()
            .add(SPEAKER_PARAM_NAME,    55.7, shooterSpeakerCloseVelocity, tiltSpeakerCloseAngle)
            .add("Speaker1ft",          68.2, 90.0, 60.0)    
            .add("Speaker2ft",          80.0, 90.0, 46.0)    
            .add("Speaker3ft",          92.0, 90.0, 43.0)    
            .add("Speaker4ft",          104.0, 90.0, 40.0)   
            .add("Speaker5ft",          117.2, 90.0, 37.0)   
            .add("Speaker6ft",          129.4, 90.0, 35.0)
            // TODO: CodeReview: This entry looks wrong. The angle is going into the wrong direction.
            .add("Speaker7ft",          139.0, 90.0, 38.0);
        public static final ShootParamTable.Params speakerParams = speakerShootParamTable.get(SPEAKER_PARAM_NAME);
    }   //class Shooter

    public static class Climber
    {
        public static final int motorCandId                     = HWConfig.CANID_CLIMBER_MOTOR;
        public static final boolean motorInverted               = false;
        public static final double gearRatio                    = 60.0;
        public static final double pulleyDiameter               = 1.88;
        public static final double posScale                     = pulleyDiameter * Math.PI / gearRatio;
        public static final PidCoefficients posPidCoeff         = new PidCoefficients(1.0, 0.0, 0.0);
        public static final double posPidTolerance              = 0.1;  // in inches.

        public static final double maxHeight                    = 9.5;
        public static final double minHeight                    = 0.0;
        public static final double climbPowerComp               = 0.3;  //TODO: tune
        public static final double calPower                     = -0.3;
    }   //class Climber

}   //class RobotParams
