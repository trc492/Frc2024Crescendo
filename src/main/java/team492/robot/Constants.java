package team492.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import team492.RobotParams;
import team492.robot.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        // public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        //     COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(RobotParams.ROBOT_WHEELBASE_WIDTH);
        public static final double wheelBase = Units.inchesToMeters(RobotParams.ROBOT_WHEELBASE_LENGTH);
        public static final double wheelCircumference = RobotParams.SWERVE_DRIVE_WHEEL_CIRCUMFERENCE;

        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = RobotParams.SWERVE_DRIVE_GEAR_RATIO;
        public static final double angleGearRatio = RobotParams.SWERVE_STEER_GEAR_RATIO;

        /* Motor Inverts */
        // Our code allows motors to be inverted individually, their code assumes all motors must be the same.
        // Steer motors should all be the same but drive motors could be different depending on how we zero align them.
        public static final boolean angleMotorInvert = true;
        public static final boolean driveMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean angleEncoderInvert = false;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = RobotParams.DRIVE_RAMP_RATE;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = RobotParams.SWERVE_STEER_KP;
        public static final double angleKI = RobotParams.SWERVE_STEER_KI;
        public static final double angleKD = RobotParams.SWERVE_STEER_KD;
        public static final double angleKF = RobotParams.SWERVE_STEER_KF;

        /* Drive Motor PID Values */
        public static final double driveKP = RobotParams.SWERVE_DRIVE_KP;
        public static final double driveKI = RobotParams.SWERVE_DRIVE_KI;
        public static final double driveKD = RobotParams.SWERVE_DRIVE_KD; //.6
        public static final double driveKF = RobotParams.SWERVE_DRIVE_KF; //.91

        /* Drive Motor Characterization Values -
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.25242 / 12); //TODO: This must be tuned to specific robot // 0.23
        public static final double driveKV = (3.9982 / 12); // 1.476
        public static final double driveKA = (0.43183/ 12); // 0.3 //.85

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = Units.inchesToMeters(RobotParams.ROBOT_MAX_VELOCITY);

        /** Radians per Second */
        public static final double maxAngularVelocity = Units.degreesToRadians(RobotParams.ROBOT_MAX_TURN_RATE);

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Left Front Module - Module 0 */
        public static final class lfConsts {
            public static final int driveMotorID = RobotParams.CANID_LFDRIVE_MOTOR;
            public static final int angleMotorID = RobotParams.CANID_LFSTEER_MOTOR;
            public static final int angleEncoderID = RobotParams.CANID_LFSTEER_ENCODER;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, angleEncoderID, angleOffset);
        }

        /* Right Front Module - Module 1 */
        public static final class rfConsts {
            public static final int driveMotorID = RobotParams.CANID_RFDRIVE_MOTOR;
            public static final int angleMotorID = RobotParams.CANID_RFSTEER_MOTOR;
            public static final int angleEncoderID = RobotParams.CANID_RFSTEER_ENCODER;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, angleEncoderID, angleOffset);
        }

        /* Left Back Module - Module 2 */
        public static final class lbConsts {
            public static final int driveMotorID = RobotParams.CANID_LBDRIVE_MOTOR;
            public static final int angleMotorID = RobotParams.CANID_LBSTEER_MOTOR;
            public static final int angleEncoderID = RobotParams.CANID_LBSTEER_ENCODER;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, angleEncoderID, angleOffset);
        }

        /* Right Back Module - Module 3 */
        public static final class rbConsts {
            public static final int driveMotorID = RobotParams.CANID_RBDRIVE_MOTOR;
            public static final int angleMotorID = RobotParams.CANID_RBSTEER_MOTOR;
            public static final int angleEncoderID = RobotParams.CANID_RBSTEER_ENCODER;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, angleEncoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 4.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.0;//0.00149
        public static final double kPYController = 0.0;
        public static final double  kPThetaController = 0.0;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
