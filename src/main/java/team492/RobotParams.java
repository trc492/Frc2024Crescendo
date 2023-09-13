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
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import team492.drivebases.RobotDrive.DriveMode;

/**
 * This class contains parameters and preferences related to all robot operations.
 */
public class RobotParams
{
    //
    // Robot preferences.
    //
    public static class Preferences
    {
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
        public static final boolean useVision                   = false;
        public static final boolean useLimeLightVision          = false;
        public static final boolean usePhotonVision             = false;
        public static final boolean useOpenCvVision             = false;
        public static final boolean useStreamCamera             = false;
        // Robot
        public static final boolean noRobot                     = false;
        // Drive Base
        public static final boolean useExternalOdometry         = false;
        public static final boolean useVelocityControl          = false;
        public static final boolean useGyroAssist               = false;
        public static final boolean useAntiTipping              = false;
        public static final boolean useSteeringCANCoder         = false;
        public static final boolean useSteeringAnalogEncoder    = true;
        
        // Subsystems
        public static final boolean useSubsystems               = true;
        // Miscellaneous
        public static final boolean useTraceLog                 = true;
        // Status Update
        public static final boolean doStatusUpdate              = true;
        public static final boolean showLoopTime                = false;

        public static final boolean showPowerConsumption        = false;

        public static final boolean showDriveBase               = false;
        public static final boolean showPurePursuitDrive        = false;
        public static final boolean showPidDrive                = false;

        public static final boolean showVision                  = false;
        public static final boolean showLimeLight               = false;
        public static final boolean showPhoton                  = false;
        public static final boolean showOpenCv                  = false;

        public static final boolean showSubsystems              = false;
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
    public static final double ROBOT_WHEELBASE_LENGTH           = 25.625;   // Required by swerve drive base.
    //
    // Robot starting positions.
    //
    public static final double STARTPOS_BLUE_Y                  = ROBOT_LENGTH/2.0;
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
    public static final int CANID_LBSTEER_ENCODER               = 25;
    public static final int CANID_RBSTEER_ENCODER               = 26;

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

    public static final double BATTERY_NOMINAL_VOLTAGE          = 12.0;
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
    public static final int CAMERA_IMAGE_WIDTH                  = 320;      // in pixels
    public static final int CAMERA_IMAGE_HEIGHT                 = 240;      // in pixels
    // Camera location on robot.
    public static final double CAMERA_Y_OFFSET                  = 9.5;      // Inches from the center of the robot
    public static final double CAMERA_X_OFFSET                  = 0.0;      // Exactly centered
    public static final double CAMERA_HEIGHT                    = 43.625;   // Inches from the floor
    public static final double CAMERA_PITCH                     = -37.8528213888;   // degrees from horizontal
    public static final double CAMERA_YAW                       = -2.9126252095;    // degrees from front
    public static final Transform3d CAMERA_TRANSFORM3D          = new Transform3d(
        new Translation3d(CAMERA_Y_OFFSET*TrcUtil.METERS_PER_INCH, -CAMERA_X_OFFSET*TrcUtil.METERS_PER_INCH,
                          CAMERA_HEIGHT*TrcUtil.METERS_PER_INCH),
        new Rotation3d(0.0, Math.toRadians(-CAMERA_PITCH), Math.toRadians(-CAMERA_YAW)));
    // Camera: Logitech C310 (not used)
    public static final double WEBCAM_FX                        = 821.993;  // in pixels
    public static final double WEBCAM_FY                        = 821.993;  // in pixels
    public static final double WEBCAM_CX                        = 330.489;  // in pixels
    public static final double WEBCAM_CY                        = 248.997;  // in pixels

    public static final double CAMERA_DATA_TIMEOUT              = 0.5;      // 500ms
    public static final double VISION_TARGET_HEIGHT             = 104.0;    // Inches from the floor (not used)
    public static final double APRILTAG_SIZE                    = 6.0 / TrcUtil.INCHES_PER_METER;   //  in meters
    // Homography measurements.
    // Camera rect in inches.
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X      = 0.0;
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y      = 120.0;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X     = CAMERA_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y     = 120.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X   = 0.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y   = CAMERA_IMAGE_HEIGHT - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X  = CAMERA_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y  = CAMERA_IMAGE_HEIGHT - 1;
    public static final TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
        RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);
    // World rect in inches.
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_X       = -12.5625;
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y       = 48.0 - ROBOT_LENGTH/2.0 + CAMERA_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X      = 11.4375;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y      = 44.75 - ROBOT_LENGTH/2.0 + CAMERA_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X    = -2.5625;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y    = 21.0 - ROBOT_LENGTH/2.0 + CAMERA_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X   = 2.5626;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y   = 21.0 - ROBOT_LENGTH + CAMERA_Y_OFFSET;
    public static final TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
        RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_X, RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_Y,
        RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_Y,
        RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
        RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);
    //
    // DriveBase subsystem.
    //
    public static final String LFDRIVE_MOTOR_NAME               = "lfDriveMotor";
    public static final String RFDRIVE_MOTOR_NAME               = "rfDriveMotor";
    public static final String LBDRIVE_MOTOR_NAME               = "lbDriveMotor";
    public static final String RBDRIVE_MOTOR_NAME               = "rbDriveMotor";
    public static final boolean LFDRIVE_MOTOR_INVERTED          = true;
    public static final boolean RFDRIVE_MOTOR_INVERTED          = false;
    public static final boolean LBDRIVE_MOTOR_INVERTED          = true;
    public static final boolean RBDRIVE_MOTOR_INVERTED          = false;
    public static final String LFSTEER_ENCODER_NAME             = "lfSteerEncoder";
    public static final String RFSTEER_ENCODER_NAME             = "rfSteerEncoder";
    public static final String LBSTEER_ENCODER_NAME             = "lbSteerEncoder";
    public static final String RBSTEER_ENCODER_NAME             = "rbSteerEncoder";
    public static final boolean LFSTEER_ENCODER_INVERTED        = false;
    public static final boolean RFSTEER_ENCODER_INVERTED        = false;
    public static final boolean LBSTEER_ENCODER_INVERTED        = false;
    public static final boolean RBSTEER_ENCODER_INVERTED        = false;
    public static final String LFSTEER_MOTOR_NAME               = "lfSteerMotor";
    public static final String RFSTEER_MOTOR_NAME               = "rfSteerMotor";
    public static final String LBSTEER_MOTOR_NAME               = "lbSteerMotor";
    public static final String RBSTEER_MOTOR_NAME               = "rbSteerMotor";
    public static final boolean LFSTEER_INVERTED                = false;
    public static final boolean RFSTEER_INVERTED                = false;
    public static final boolean LBSTEER_INVERTED                = false;
    public static final boolean RBSTEER_INVERTED                = false;
    public static final String LFSWERVE_MODULE_NAME             = "lfWheel";
    public static final String RFSWERVE_MODULE_NAME             = "rfWheel";
    public static final String LBSWERVE_MODULE_NAME             = "lbWheel";
    public static final String RBSWERVE_MODULE_NAME             = "rbWheel";
    public static final DriveMode ROBOT_DRIVE_MODE              = DriveMode.HOLONOMIC_MODE;
    // West Coast Drive Base (not used);
    public static final double WCD_INCHES_PER_COUNT             = 2.2421;
    public static final double WCD_KP                           = 0.011;
    public static final double WCD_KI                           = 0.0;
    public static final double WCD_KD                           = 0.0013;
    public static final double WCD_KF                           = 0.0;
    public static final double WCD_TOLERANCE                    = 2.0;
    // Mecanum Drive Base (not used).
    public static final double MECANUM_X_INCHES_PER_COUNT       = 2.2421;
    public static final double MECANUM_X_KP                     = 0.011;
    public static final double MECANUM_X_KI                     = 0.0;
    public static final double MECANUM_X_KD                     = 0.0013;
    public static final double MECANUM_X_KF                     = 0.0;
    public static final double MECANUM_X_TOLERANCE              = 2.0;

    public static final double MECANUM_Y_INCHES_PER_COUNT       = 2.2421;
    public static final double MECANUM_Y_KP                     = 0.011;
    public static final double MECANUM_Y_KI                     = 0.0;
    public static final double MECANUM_Y_KD                     = 0.0013;
    public static final double MECANUM_Y_KF                     = 0.0;
    public static final double MECANUM_Y_TOLERANCE              = 2.0;

    public static final double SWERVE_INCHES_PER_COUNT          = 9.072106867127145344367826764411e-4;
    public static final double SWERVE_KP                        = 0.02;
    public static final double SWERVE_KI                        = 0.0;
    public static final double SWERVE_KD                        = 0.0;
    public static final double SWERVE_KF                        = 0.0;
    public static final double SWERVE_IZONE                     = 5.0;
    public static final double SWERVE_TOLERANCE                 = 2.0;

    public static final double GYRO_TURN_KP                     = 0.012;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.0;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_IZONE                  = 10.0;
    public static final double GYRO_TURN_TOLERANCE              = 2.0;

    public static final double GYRO_ASSIST_TURN_GAIN            = 0.1;

    // Not tuned (not used).
    public static final double X_TIPPING_KP                     = 0.01;
    public static final double X_TIPPING_KI                     = 0.0;
    public static final double X_TIPPING_KD                     = 0.0;
    public static final double X_TIPPING_TOLERANCE              = 10.0;
    public static final double X_TIPPING_SETTLING_TIME          = 0.2;

    public static final double Y_TIPPING_KP                     = 0.01;
    public static final double Y_TIPPING_KI                     = 0.0;
    public static final double Y_TIPPING_KD                     = 0.0;
    public static final double Y_TIPPING_TOLERANCE              = 10.0;
    public static final double Y_TIPPING_SETTLING_TIME          = 0.2;

    public static final double ROBOT_MAX_VELOCITY               = 172.9;
    public static final double ROBOT_MAX_ACCELERATION           = 799.1;
    public static final double ROBOT_MAX_TURN_RATE              = 562.5;
    public static final double ROBOT_VEL_KP                     = 0.0;
    public static final double ROBOT_VEL_KI                     = 0.0;
    public static final double ROBOT_VEL_KD                     = 0.0;
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    public static final double ROBOT_VEL_KF                     = 1.0 / ROBOT_MAX_VELOCITY;

    public static final double DRIVE_SLOW_SCALE                 = 0.5;
    public static final double TURN_SLOW_SCALE                  = 0.3;
    public static final double DRIVE_NORMAL_SCALE               = 0.75;
    public static final double TURN_NORMAL_SCALE                = 0.6;

    public static final double DRIVE_MAX_XPID_POWER             = 0.5;
    public static final double DRIVE_MAX_YPID_POWER             = 0.6;
    public static final double DRIVE_MAX_TURNPID_POWER          = 1.0;

    public static final double DRIVE_MAX_XPID_RAMP_RATE         = 0.5;
    public static final double DRIVE_MAX_YPID_RAMP_RATE         = 0.5;
    public static final double DRIVE_MAX_TURNPID_RAMP_RATE      = 1.0;

    public static final double DRIVE_RAMP_RATE                  = 0.2;

    // Applicable only for Swerve Drive.
    public static final double CANCODER_CPR                     = 4096.0;
    public static final double FALCON_CPR                       = 2048.0;
    public static final double FALCON_MAX_RPM                   = 6380.0;
    public static final double STEER_GEAR_RATIO                 = (24.0/12.0) * (72.0/14.0);
    public static final double STEER_MOTOR_CPR                  = FALCON_CPR * STEER_GEAR_RATIO;
    public static final double STEER_DEGREES_PER_COUNT          = 360.0 / STEER_MOTOR_CPR;
    // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
    public static final double STEER_MAX_VEL                    = (FALCON_MAX_RPM*0.81/STEER_GEAR_RATIO/60.0)*360.0;

    public static final double STEER_MAX_REQ_VEL                = 1000.0;   // deg/sec. max commanded velocity, not necessarily max vel
    public static final double STEER_MAX_ACCEL                  = 5000.0;   // deg/sec^2

    // Zeroes are normalized offsets which are in the unit of percentage revolution (0.0 to 1.0).
    // This is a backup if file is not found: LF, RF, LB, RB.
    public static final double[] STEER_ZEROS                    = new double[] {0.493703, 0.278641, 0.409850, 0.443877};

    public static final double STEER_MAX_VEL_COUNT_PER_100MS    = (STEER_MAX_VEL / STEER_DEGREES_PER_COUNT) / 10.0;
    // public static final TrcPidController.PidCoefficients magicSteerCoeff =
    //     new TrcPidController.PidCoefficients(2.0, 0.01, 0.0, 1023.0 / STEER_MAX_VEL_COUNT_PER_100MS, 5.0 / STEER_DEGREES_PER_COUNT);
    public static final double STEER_KP                         = 1.1;
    public static final double STEER_KI                         = 0.0;
    public static final double STEER_KD                         = 14.0;
    // kF set to Motion Magic recommendation.
    public static final double STEER_KF                         = 0.0;//1023.0 / STEER_MAX_VEL_COUNT_PER_100MS;
    // iZone set to within 5 steering degrees.
    public static final double STEER_IZONE                      = 0.0;//5.0 / STEER_DEGREES_PER_COUNT;
    public static final TrcPidController.PidCoefficients steerCoeffs =
        new TrcPidController.PidCoefficients(STEER_KP, STEER_KI, STEER_KD, STEER_KF, STEER_IZONE);

    public static final double PPD_FOLLOWING_DISTANCE           = 12.0;
    public static final double PPD_POS_TOLERANCE                = 1.0;
    public static final double PPD_TURN_TOLERANCE               = 2.0;
    public static final double PPD_MOVE_DEF_OUTPUT_LIMIT        = 0.5;
    public static final double PPD_ROT_DEF_OUTPUT_LIMIT         = 0.5;
    //
    // Other subsystems.
    //

}   //class RobotParams
