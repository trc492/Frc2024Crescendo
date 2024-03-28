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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492.drivebases;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Arrays;
import java.util.Scanner;

import com.ctre.phoenix6.StatusCode;
import com.reduxrobotics.canand.CanandEventLoop;

import TrcCommonLib.trclib.TrcWatchdogMgr;
import TrcCommonLib.trclib.TrcDbgTrace.MsgLevel;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEncoder;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcSwerveDriveBase;
import TrcCommonLib.trclib.TrcSwerveModule;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcCommonLib.trclib.TrcWatchdogMgr.Watchdog;
import TrcFrcLib.frclib.FrcAHRSGyro;
import TrcFrcLib.frclib.FrcAnalogEncoder;
import TrcFrcLib.frclib.FrcCANCoder;
import TrcFrcLib.frclib.FrcCANTalonFX;
import TrcFrcLib.frclib.FrcCanandcoder;
import TrcFrcLib.frclib.FrcPdp;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import team492.Robot;
import team492.RobotParams;
import team492.RobotParams.SteerEncoderType;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class SwerveDrive extends RobotDrive
{
    private static final String moduleName = SwerveDrive.class.getSimpleName();

    //
    // Swerve steering motors and modules.
    //
    public final RobotParams.SwerveDriveBase driveBaseParams;
    public final TrcMotor[] steerMotors;
    public final TrcEncoder[] steerEncoders;
    public final TrcSwerveModule[] swerveModules;
    public final SwerveDriveOdometry swerveOdometry;
    private final SimpleMotorFeedforward driveFeedForward;

    private double[] steerZeros = new double[4];
    private int steerZeroCalibrationCount = 0;
    private String xModeOwner = null;
    private boolean steerEncodersSynced = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     * @param driveBaseParams specifies the drivebase parameters.
     */
    public SwerveDrive(Robot robot, RobotParams.SwerveDriveBase driveBaseParams)
    {
        super(robot);

        this.driveBaseParams = driveBaseParams;
        driveMotors = createMotors(
            MotorType.CanTalonFx, false, driveBaseParams.driveMotorNames, driveBaseParams.driveMotorIds,
            driveBaseParams.driveMotorInverted);
        steerMotors = createMotors(
            MotorType.CanTalonFx, false, driveBaseParams.steerMotorNames, driveBaseParams.steerMotorIds,
            driveBaseParams.steerMotorInverted);
        steerEncoders = createSteerEncoders(
            driveBaseParams.steerEncoderNames,
            driveBaseParams.steerEncoderType.equals(SteerEncoderType.AnalogEncoder) ?
                driveBaseParams.steerEncoderAnalogIds : driveBaseParams.steerEncoderCanIds,
            driveBaseParams.steerEncoderInverted, readSteeringCalibrationData());
        swerveModules = createSwerveModules(
            driveBaseParams.swerveModuleNames, driveMotors, steerMotors, steerEncoders);

        swerveOdometry = new SwerveDriveOdometry(driveBaseParams.swerveKinematics, getGyroYaw(), getModulePositions());
        driveFeedForward = new SimpleMotorFeedforward(
            driveBaseParams.DRIVE_KS, driveBaseParams.DRIVE_KV, driveBaseParams.DRIVE_KA);

        driveBase = new TrcSwerveDriveBase(
            swerveModules[RobotDrive.INDEX_LEFT_FRONT], swerveModules[RobotDrive.INDEX_LEFT_BACK],
            swerveModules[RobotDrive.INDEX_RIGHT_FRONT], swerveModules[RobotDrive.INDEX_RIGHT_BACK],
            gyro, RobotParams.Robot.WHEELBASE_WIDTH, RobotParams.Robot.WHEELBASE_LENGTH);
        driveBase.setOdometryScales(driveBaseParams.DRIVE_INCHES_PER_ROT, driveBaseParams.DRIVE_INCHES_PER_ROT);

        if (RobotParams.Preferences.useAntiTipping)
        {
            driveBase.enableAntiTipping(
                new TrcPidController.PidCoefficients(
                    driveBaseParams.X_TIPPING_KP, driveBaseParams.X_TIPPING_KI, driveBaseParams.X_TIPPING_KD),
                driveBaseParams.X_TIPPING_TOLERANCE, this::getGyroRoll,
                new TrcPidController.PidCoefficients(
                    driveBaseParams.Y_TIPPING_KP, driveBaseParams.Y_TIPPING_KI, driveBaseParams.Y_TIPPING_KD),
                driveBaseParams.Y_TIPPING_TOLERANCE, this::getGyroPitch);
        }

        if (robot.pdp != null)
        {
            robot.pdp.registerEnergyUsed(
                new FrcPdp.Channel(
                    RobotParams.HWConfig.PDP_CHANNEL_LFDRIVE_MOTOR,
                    driveBaseParams.driveMotorNames[RobotDrive.INDEX_LEFT_FRONT]),
                new FrcPdp.Channel(
                    RobotParams.HWConfig.PDP_CHANNEL_LBDRIVE_MOTOR,
                    driveBaseParams.driveMotorNames[RobotDrive.INDEX_LEFT_BACK]),
                new FrcPdp.Channel(
                    RobotParams.HWConfig.PDP_CHANNEL_RFDRIVE_MOTOR,
                    driveBaseParams.driveMotorNames[RobotDrive.INDEX_RIGHT_FRONT]),
                new FrcPdp.Channel(
                    RobotParams.HWConfig.PDP_CHANNEL_RBDRIVE_MOTOR,
                    driveBaseParams.driveMotorNames[RobotDrive.INDEX_RIGHT_BACK]),
                new FrcPdp.Channel(
                    RobotParams.HWConfig.PDP_CHANNEL_LFSTEER_MOTOR,
                    driveBaseParams.steerMotorNames[RobotDrive.INDEX_LEFT_FRONT]),
                new FrcPdp.Channel(
                    RobotParams.HWConfig.PDP_CHANNEL_LBSTEER_MOTOR,
                    driveBaseParams.steerMotorNames[RobotDrive.INDEX_LEFT_BACK]),
                new FrcPdp.Channel(
                    RobotParams.HWConfig.PDP_CHANNEL_RFSTEER_MOTOR,
                    driveBaseParams.steerMotorNames[RobotDrive.INDEX_RIGHT_FRONT]),
                new FrcPdp.Channel(
                    RobotParams.HWConfig.PDP_CHANNEL_RBSTEER_MOTOR,
                    driveBaseParams.steerMotorNames[RobotDrive.INDEX_RIGHT_BACK]));
        }
        //
        // Create and initialize PID controllers.
        //
        // PID Parameters for X and Y are the same for Swerve Drive.
        xPosPidCoeff = yPosPidCoeff = new TrcPidController.PidCoefficients(
            driveBaseParams.DRIVE_KP, driveBaseParams.DRIVE_KI, driveBaseParams.DRIVE_KD, driveBaseParams.DRIVE_KF,
            driveBaseParams.DRIVE_IZONE);
        turnPidCoeff = new TrcPidController.PidCoefficients(
            driveBaseParams.TURN_KP, driveBaseParams.TURN_KI, driveBaseParams.TURN_KD, driveBaseParams.TURN_KF,
            driveBaseParams.TURN_IZONE);
        velPidCoeff = new TrcPidController.PidCoefficients(
            driveBaseParams.ROBOT_VEL_KP, driveBaseParams.ROBOT_VEL_KI, driveBaseParams.ROBOT_VEL_KD,
            driveBaseParams.ROBOT_VEL_KF);

        pidDrive = new TrcPidDrive(
            "pidDrive", driveBase,
            xPosPidCoeff, driveBaseParams.DRIVE_TOLERANCE, driveBase::getXPosition,
            yPosPidCoeff, driveBaseParams.DRIVE_TOLERANCE, driveBase::getYPosition,
            turnPidCoeff, driveBaseParams.TURN_TOLERANCE, driveBase::getHeading);

        TrcPidController xPidCtrl, yPidCtrl, turnPidCtrl, velPidCtrl;
        xPidCtrl = pidDrive.getXPidCtrl();
        xPidCtrl.setOutputLimit(driveBaseParams.DRIVE_MAX_XPID_POWER);
        xPidCtrl.setRampRate(driveBaseParams.DRIVE_MAX_XPID_RAMP_RATE);

        yPidCtrl = pidDrive.getYPidCtrl();
        yPidCtrl.setOutputLimit(driveBaseParams.DRIVE_MAX_YPID_POWER);
        yPidCtrl.setRampRate(driveBaseParams.DRIVE_MAX_YPID_RAMP_RATE);

        turnPidCtrl = pidDrive.getTurnPidCtrl();
        turnPidCtrl.setOutputLimit(driveBaseParams.DRIVE_MAX_TURNPID_POWER);
        turnPidCtrl.setRampRate(driveBaseParams.DRIVE_MAX_TURNPID_RAMP_RATE);
        turnPidCtrl.setAbsoluteSetPoint(true);

        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setTraceLevel(MsgLevel.INFO, false, false, false);

        xPidCtrl = new TrcPidController("purePursuitDrive.xPosPid", xPosPidCoeff, driveBase::getXPosition);
        yPidCtrl = new TrcPidController("purePursuitDrive.yPosPid", yPosPidCoeff, driveBase::getYPosition);
        turnPidCtrl = new TrcPidController("purePursuitDrive.turnPid", turnPidCoeff, robot::getHeadingInput);
        // We are not checking velocity being onTarget, so we don't need velocity tolerance.
        velPidCtrl = new TrcPidController("purePursuitDrive.velPid", velPidCoeff, this::getVelocityInput);
        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase,
            driveBaseParams.PPD_FOLLOWING_DISTANCE, driveBaseParams.PPD_POS_TOLERANCE,
            driveBaseParams.PPD_TURN_TOLERANCE, xPidCtrl, yPidCtrl, turnPidCtrl, velPidCtrl);
        purePursuitDrive.setStallDetectionEnabled(0.1, 0.1, 1.0);
        purePursuitDrive.setMoveOutputLimit(driveBaseParams.PPD_MOVE_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setRotOutputLimit(driveBaseParams.PPD_ROT_DEF_OUTPUT_LIMIT);
        // purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setTraceLevel(MsgLevel.INFO, false, false, false);
    }   //SwerveDrive

    /**
     * This method is called by the Velocity PID controller to get the polar magnitude of the robot's velocity.
     *
     * @return robot's velocity magnitude.
     */
    private double getVelocityInput()
    {
        return TrcUtil.magnitude(driveBase.getXVelocity(), driveBase.getYVelocity());
    }   //getVelocityInput

    /**
     * This method creates an array of steer encoders for each steer motor and configure them.
     *
     * @param names specifies an array of names for each steer encoder.
     * @param encoderIds specifies an array of IDs for each steer encoder (CAN IDs for CAN coders, analog channels
     *        for analog encoders).
     * @param inverted specifies an array of boolean indicating if the encoder needs to be inverted.
     * @param steerZeros specifies an array of zero positions for each steer encoder.
     * @return created array of steer encoders.
     */
    private TrcEncoder[] createSteerEncoders(String[] names, int[] encoderIds, boolean[] inverted, double[] steerZeros)
    {
        TrcEncoder[] encoders = null;

        if (driveBaseParams.steerEncoderType.equals(SteerEncoderType.CANCoder))
        {
            encoders = new FrcCANCoder[names.length];
            for (int i = 0; i < names.length; i++)
            {
                FrcCANCoder canCoder = new FrcCANCoder(names[i], encoderIds[i]);
                try
                {
                    canCoder.resetFactoryDefault();
                    // Configure the sensor direction to match the steering motor direction.
                    canCoder.setInverted(inverted[i]);
                    canCoder.setAbsoluteRange(true);
                    // CANCoder is already normalized to the range of 0 to 1.0 for a revolution
                    // (revolution per count).
                    canCoder.setScaleAndOffset(1.0, 0.0, steerZeros[i]);
                    encoders[i] = canCoder;
                }
                finally
                {
                    canCoder.close();
                }
            }
        }
        else if (driveBaseParams.steerEncoderType.equals(SteerEncoderType.Canandcoder))
        {
            CanandEventLoop.getInstance();
            encoders = new FrcCanandcoder[names.length];
            for (int i = 0; i < names.length; i++)
            {
                try (FrcCanandcoder canandcoder = new FrcCanandcoder(names[i], encoderIds[i]))
                {
                    canandcoder.resetFactoryDefaults(false);
                    // Configure the sensor direction to match the steering motor direction.
                    canandcoder.setInverted(inverted[i]);
                    // Canandcoder is already normalized to the range of 0 to 1.0 for a revolution
                    // (revolution per count).
                    canandcoder.setScaleAndOffset(1.0, 0.0, steerZeros[i]);
                    encoders[i] = canandcoder;
                }
            }
        }
        else if (driveBaseParams.steerEncoderType.equals(SteerEncoderType.AnalogEncoder))
        {
            encoders = new TrcEncoder[names.length];
            for (int i = 0; i < names.length; i++)
            {
                TrcEncoder analogEncoder = new FrcAnalogEncoder(names[i], encoderIds[i]).getAbsoluteEncoder();
                analogEncoder.setInverted(inverted[i]);
                // Analog Encoder is already normalized to the range of 0 to 1.0 for a revolution
                // (revolution per count).
                analogEncoder.setScaleAndOffset(1.0, 0.0, steerZeros[i]);
                encoders[i] = analogEncoder;
            }
        }
        else
        {
            throw new IllegalArgumentException("Encoder type must either CANCoder, Canandcoder or AnalogEncoder.");
        }

        return encoders;
    }   //createSteerEncoders

    /**
     * This method reads the absolute steering encoder and synchronize the steering motor encoder with it.
     *
     * @param index specifies the swerve module index.
     */
    private void syncSteerEncoder(int index)
    {
        // getPosition returns a value in the range of 0 to 1.0 of one revolution.
        double motorEncoderPos =
            steerEncoders[index].getScaledPosition() * driveBaseParams.STEER_GEAR_RATIO;
        StatusCode statusCode = ((FrcCANTalonFX) steerMotors[index]).motor.setPosition(motorEncoderPos);
        if (statusCode != StatusCode.OK)
        {
            robot.globalTracer.traceWarn(
                moduleName,
                driveBaseParams.swerveModuleNames[index] + ": TalonFx.setPosition failed (code=" + statusCode +
                ", pos=" + motorEncoderPos + ").");
        }

        double actualEncoderPos = ((FrcCANTalonFX) steerMotors[index]).motor.getPosition().getValueAsDouble();
        if (Math.abs(motorEncoderPos - actualEncoderPos) > 0.1)
        {
            robot.globalTracer.traceWarn(
                driveBaseParams.swerveModuleNames[index],
                "Steer encoder out-of-sync (expected=" + motorEncoderPos + ", actual=" + actualEncoderPos + ")");
        }
    }   //syncSteerEncoder

    /**
     * This method resets the steer motor encoder for emergency steering alignment in case the absolute encoder is
     * malfunctioning.
     */
    public void resetSteerEncoders()
    {
        for (TrcMotor motor: steerMotors)
        {
            FrcCANTalonFX steerMotor = (FrcCANTalonFX) motor;
            StatusCode statusCode = steerMotor.motor.setPosition(0.0);
            if (statusCode != StatusCode.OK)
            {
                robot.globalTracer.traceWarn(
                    moduleName,
                    steerMotor.toString() + ": TalonFx.setPosition failed (code=" + statusCode + ").");
            }
        }
    }   //resetSteerEncoders

    /**
     * This method creates an array of swerve modules and configure them.
     *
     * @param names specifies an array of names for each swerve module.
     * @param driveMotors specifies an array of drive motors for each swerve module.
     * @param steerMotors specifies an array of steer motors for each swerve module.
     * @param steerEncoders specifies an array of steer encoders for each swerve module.
     * @return created array of swerve modules.
     */
    private TrcSwerveModule[] createSwerveModules(
        String[] names, TrcMotor[] driveMotors, TrcMotor[] steerMotors, TrcEncoder[] steerEncoders)
    {
        TrcSwerveModule[] modules = new TrcSwerveModule[names.length];

        for (int i = 0; i < names.length; i++)
        {
            driveMotors[i].setBrakeModeEnabled(true);
            // Do not scale motor odometry. DriveBase is taking care of scaling.
            driveMotors[i].setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);

            driveMotors[i].setCloseLoopRampRate(0.02);
            driveMotors[i].setCurrentLimit(40.0, 45.0, 0.2);
            driveMotors[i].setStatorCurrentLimit(55.0);

            steerMotors[i].setBrakeModeEnabled(false);
            steerMotors[i].setPositionSensorScaleAndOffset(driveBaseParams.STEER_DEGREES_PER_COUNT, 0.0);
            steerMotors[i].setPositionPidParameters(
                driveBaseParams.steerPosCoeffs, driveBaseParams.steerPosTolerance);
            steerMotors[i].setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
            syncSteerEncoder(i);

            // We have already synchronized the TalonFx internal encoder with the zero adjusted absolute encoder, so
            // motor does not need to compensate for zero position.
            modules[i] = new TrcSwerveModule(names[i], driveMotors[i], steerMotors[i]);
        }

        return modules;
    }   //createSwerveModules

    /**
     * This method displays the steering absolute encoder and internal motor encoder values on the dashboard for
     * debuggig purpose.
     *
     * @param lineNum specifies the starting line number on the dashboard to display the info.
     */
    public void displaySteerEncoders(int lineNum)
    {
        double lfSteerAbsEnc = steerEncoders[RobotDrive.INDEX_LEFT_FRONT].getScaledPosition()*360.0;
        // if (lfSteerAbsEnc > 90.0) lfSteerAbsEnc = 180.0 - lfSteerAbsEnc;
        double rfSteerAbsEnc = steerEncoders[RobotDrive.INDEX_RIGHT_FRONT].getScaledPosition()*360.0;
        // if (rfSteerAbsEnc > 90.0) rfSteerAbsEnc = 180.0 - rfSteerAbsEnc;
        double lbSteerAbsEnc = steerEncoders[RobotDrive.INDEX_LEFT_BACK].getScaledPosition()*360.0;
        // if (lbSteerAbsEnc > 90.0) lbSteerAbsEnc = 180.0 - lbSteerAbsEnc;
        double rbSteerAbsEnc = steerEncoders[RobotDrive.INDEX_RIGHT_BACK].getScaledPosition()*360.0;
        // if (rbSteerAbsEnc > 90.0) rbSteerAbsEnc = 180.0 - rbSteerAbsEnc;
        double lfSteerEnc =
            (360.0 * steerMotors[RobotDrive.INDEX_LEFT_FRONT].getMotorPosition() /
             driveBaseParams.STEER_GEAR_RATIO) % 360.0;
        double rfSteerEnc =
            (360.0 * steerMotors[RobotDrive.INDEX_RIGHT_FRONT].getMotorPosition() /
             driveBaseParams.STEER_GEAR_RATIO) % 360.0;
        double lbSteerEnc =
            (360.0 * steerMotors[RobotDrive.INDEX_LEFT_BACK].getMotorPosition() /
             driveBaseParams.STEER_GEAR_RATIO) % 360.0;
        double rbSteerEnc =
            (360.0 * steerMotors[RobotDrive.INDEX_RIGHT_BACK].getMotorPosition() /
             driveBaseParams.STEER_GEAR_RATIO) % 360.0;

        robot.dashboard.displayPrintf(
            lineNum, "SteerEnc: lf=%6.1f/%6.1f, rf=%6.1f/%6.1f, lb=%6.1f/%6.1f, rb=%6.1f/%6.1f",
            lfSteerAbsEnc, lfSteerEnc, rfSteerAbsEnc, rfSteerEnc,
            lbSteerAbsEnc, lbSteerEnc, rbSteerAbsEnc, rbSteerEnc);
        lineNum++;
        robot.dashboard.displayPrintf(
            lineNum, "SteerErr: lf=%6.3f, rf=%6.3f, lb=%6.3f, rb=%6.3f",
            lfSteerAbsEnc - lfSteerEnc, rfSteerAbsEnc - rfSteerEnc,
            lbSteerAbsEnc - lbSteerEnc, rbSteerAbsEnc - rbSteerEnc);
        lineNum++;

        robot.dashboard.putNumber("Graphs/lfSteerAbsPos", lfSteerAbsEnc);
        robot.dashboard.putNumber("Graphs/rfSteerAbsPos", rfSteerAbsEnc);
        robot.dashboard.putNumber("Graphs/lbSteerAbsPos", lbSteerAbsEnc);
        robot.dashboard.putNumber("Graphs/rbSteerAbsPos", rbSteerAbsEnc);
        robot.dashboard.putNumber("Graphs/lfSteerPos", lfSteerEnc);
        robot.dashboard.putNumber("Graphs/rfSteerPos", rfSteerEnc);
        robot.dashboard.putNumber("Graphs/lbSteerPos", lbSteerEnc);
        robot.dashboard.putNumber("Graphs/rbSteerPos", rbSteerEnc);
    }   //displaySteerEncoders

    /**
     * This method is called to set all swerve wheels to zero degrees.
     *
     * @param optimize specifies true to optimize the shortest way to point the wheels forward, could end up at
     *        180-degree instead of zero, false to set wheel angle to absolute zero.
     */
    public void setSteerAngleZero(boolean optimize)
    {
        for (TrcSwerveModule module: swerveModules)
        {
            module.setSteerAngle(0.0, optimize);
        }
    }   //setSteerAngleZero

    /**
     * This method checks if the steer motor internal encoders are in sync with the absolute encoders. If not, it will
     * do a re-sync of the steer motor encoders to the absolute enocder posiitions. This method can be called multiple
     * times but it will only perform the re-sync the first time it's called unless forceSync is set to true.
     *
     * @param forceSync specifies true to force performing the encoder resync, false otherwise.
     */
    public void syncSteerEncoders(boolean forceSync)
    {
        final double encErrThreshold = 0.01;
        final double timeout = 0.5;

        if (!steerEncodersSynced || forceSync)
        {
            final Watchdog watchdog = TrcWatchdogMgr.getWatchdog();
            double expiredTime = TrcTimer.getCurrentTime() + timeout;
            boolean onTarget = false;

            watchdog.pauseWatch();
            setSteerAngleZero(false);
            TrcTimer.sleep(200);
            while (!onTarget && TrcTimer.getCurrentTime() < expiredTime)
            {
                onTarget = true;
                for (int i = 0; i < steerMotors.length; i++)
                {
                    double steerPos = steerMotors[i].getMotorPosition();
                    if (Math.abs(steerPos) > encErrThreshold)
                    {
                        robot.globalTracer.traceInfo(moduleName, "steerEncPos[" + i + "]=" + steerPos);
                        onTarget = false;
                        break;
                    }
                }

                if (!onTarget)
                {
                    Thread.yield();
                }
            }

            if (!onTarget)
            {
                for (int i = 0; i < steerMotors.length; i++)
                {
                    syncSteerEncoder(i);
                }
            }

            steerEncodersSynced = true;
        }
    }   //syncSteerEncoders

    /**
     * This method is called to prepare the robot base before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    @Override
    public void startMode(RunMode runMode, RunMode prevMode)
    {
        super.startMode(runMode, prevMode);
        // Set all swerve steering pointing to absolute forward to start.
        if (runMode != RunMode.TEST_MODE && runMode != RunMode.DISABLED_MODE)
        {
            setSteerAngleZero(false);
        }
    }   //startMode

    /**
     * This method is called to prepare the robot base right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    @Override
    public void stopMode(RunMode runMode, RunMode nextMode)
    {
        super.stopMode(runMode, nextMode);
        // setSteerAngleZero(false);
    }   //stopMode

    /**
     * This method starts the steering calibration.
     */
    public void startSteeringCalibration()
    {
        steerZeroCalibrationCount = 0;
        Arrays.fill(steerZeros, 0.0);
    }   //startSteeringCalibration

    /**
     * This method stops the steering calibration and saves the calibration data to a file.
     */
    public void stopSteeringCalibration()
    {
        for (int i = 0; i < steerZeros.length; i++)
        {
            steerZeros[i] /= steerZeroCalibrationCount;
        }
        steerZeroCalibrationCount = 0;
        saveSteeringCalibrationData(steerZeros);
    }   //stopSteeringCalibration

    /**
     * This method is called periodically to sample the steer encoders for averaging the zero position data.
     */
    public void runSteeringCalibration()
    {
        for (int i = 0; i < steerZeros.length; i++)
        {
            steerZeros[i] += steerEncoders[i].getRawPosition();
        }
        steerZeroCalibrationCount++;
    }   //runSteeringCalibration

    /**
     * This method saves the calibration data to a file on the Robot Controller.
     *
     * @param steerZeros specifies the steering zero calibration data to be saved.
     */
    public void saveSteeringCalibrationData(double[] steerZeros)
    {
        try (PrintStream out = new PrintStream(new FileOutputStream(
            RobotParams.TEAM_FOLDER_PATH + "/" + RobotParams.SwerveDriveBase.STEER_ZERO_CAL_FILE)))
        {
            for (int i = 0; i < driveBaseParams.steerMotorNames.length; i++)
            {
                out.println(driveBaseParams.steerMotorNames[i] + ": " + steerZeros[i]);
            }
            out.close();
            robot.globalTracer.traceInfo(
                moduleName,
                "SteeringCalibrationData" + Arrays.toString(driveBaseParams.steerMotorNames) +
                "=" + Arrays.toString(steerZeros));
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }   //saveSteeringCalibrationData

    /**
     * This method reads the steering zero calibration data from the calibration data file.
     *
     * @return calibration data of all four swerve modules.
     */
    public double[] readSteeringCalibrationData()
    {
        String line = null;

        try (Scanner in = new Scanner(new FileReader(
            RobotParams.TEAM_FOLDER_PATH + "/" + RobotParams.SwerveDriveBase.STEER_ZERO_CAL_FILE)))
        {
            double[] steerZeros = new double[steerMotors.length];

            for (int i = 0; i < steerMotors.length; i++)
            {
                line = in.nextLine();
                int colonPos = line.indexOf(':');
                String name = colonPos == -1? null: line.substring(0, colonPos);

                if (name == null || !name.equals(driveBaseParams.steerMotorNames[i]))
                {
                    throw new RuntimeException("Invalid steer motor name in line " + line);
                }

                steerZeros[i] = Double.parseDouble(line.substring(colonPos + 1));
            }
            robot.globalTracer.traceInfo(
                moduleName,
                "SteeringCalibrationData" + Arrays.toString(driveBaseParams.steerMotorNames) +
                "=" + Arrays.toString(steerZeros));

            return steerZeros;
        }
        catch (FileNotFoundException e)
        {
            robot.globalTracer.traceWarn(
                moduleName, "Steering calibration data file not found, using built-in defaults.");
            return driveBaseParams.STEER_ZEROS;
        }
        catch (NumberFormatException e)
        {
            throw new RuntimeException("Invalid zero position value: " + line);
        }
        catch (RuntimeException e)
        {
            throw new RuntimeException("Invalid steer motor name: " + line);
        }
    }   //readSteeringCalibrationData

    /**
     * This method set all the wheels into an X configuration so that nobody can bump us out of position. If owner
     * is specifies, it will acquire execlusive ownership of the drivebase on behalf of the specified owner. On
     * disable, it will release the ownership.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param enabled   specifies true to enable anti-defense mode, false to disable.
     */
    public void setXModeEnabled(String owner, boolean enabled)
    {
        if (enabled)
        {
            if (owner != null && !driveBase.hasOwnership(owner) && driveBase.acquireExclusiveAccess(owner))
            {
                xModeOwner = owner;
            }

            ((TrcSwerveDriveBase) driveBase).setXMode(owner);
        }
        else if (xModeOwner != null)
        {
            driveBase.releaseExclusiveAccess(xModeOwner);
            xModeOwner = null;
        }
    }   //setXModeEnabled

    //
    // Command-based required methods.
    //

    private void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, driveBaseParams.maxSpeed);
        for (int i = 0; i < desiredStates.length; i++)
        {
            // Set steer angle.
            desiredStates[i] = SwerveModuleState.optimize(
                desiredStates[i], Rotation2d.fromRotations(steerMotors[i].getMotorPosition()));
            steerMotors[i].setMotorPosition(desiredStates[i].angle.getRotations(), null, 0.0, 0.0);
            // Set drive wheel speed.
            if (isOpenLoop)
            {
                double dutyCycle = desiredStates[i].speedMetersPerSecond / driveBaseParams.maxSpeed;
                driveMotors[i].setMotorPower(dutyCycle);
                TrcDbgTrace.globalTraceInfo(
                    "SwerveMod" + i, "DriveSpeedOpenLoop: speed=%.3f, dutyCycle=%.3f, SteerAngle=%.3f",
                    desiredStates[i].speedMetersPerSecond, dutyCycle, desiredStates[i].angle.getRotations());
            }
            else
            {
                double velocity = Conversions.MPSToRPS(
                    desiredStates[i].speedMetersPerSecond, driveBaseParams.wheelCircumference) /
                    driveBaseParams.DRIVE_GEAR_RATIO;
                double feedForward = driveFeedForward.calculate(desiredStates[i].speedMetersPerSecond);
                driveMotors[i].setMotorVelocity(velocity, 0.0, feedForward);
                TrcDbgTrace.globalTraceInfo(
                    "SwerveMod" + i,
                    "DriveSpeedClosedLoop: speed=%.3f, motorVel=%.3f, feedForward=%.3f, SteerAngle=%.3f",
                    desiredStates[i].speedMetersPerSecond, velocity, feedForward, desiredStates[i].angle.getRotations());
            }
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        setModuleStates(desiredStates, false);
    }

    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int i = 0; i < positions.length; i++)
        {
            positions[i] = new SwerveModulePosition(
                Conversions.rotationsToMeters(driveMotors[i].getMotorPosition(), driveBaseParams.wheelCircumference),
                Rotation2d.fromRotations(steerMotors[i].getMotorPosition()));
        }

        return positions;
    }

    public Pose2d getPose()
    {
        return swerveOdometry.getPoseMeters();
    }   //getPose

    public void setPose(Pose2d pose)
    {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading()
    {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading)
    {
        swerveOdometry.resetPosition(
            getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading()
    {
        swerveOdometry.resetPosition(
            getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw()
    {
        double gyroYaw = ((FrcAHRSGyro) gyro).ahrs.getYaw();
        return (driveBaseParams.invertGyro) ? Rotation2d.fromDegrees(360 - gyroYaw) : Rotation2d.fromDegrees(gyroYaw);
    }

    @Override
    public void periodic()
    {
        swerveOdometry.update(getGyroYaw(), getModulePositions());
    }

}   //class SwerveDrive
