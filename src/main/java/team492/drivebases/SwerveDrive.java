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

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import TrcCommonLib.trclib.TrcWatchdogMgr;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEncoder;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcSwerveDriveBase;
import TrcCommonLib.trclib.TrcSwerveModule;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcCommonLib.trclib.TrcWatchdogMgr.Watchdog;
import TrcFrcLib.frclib.FrcAnalogEncoder;
import TrcFrcLib.frclib.FrcCANCoder;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcPdp;
import team492.Robot;
import team492.RobotParams;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class SwerveDrive extends RobotDrive
{
    private static final boolean logPoseEvents = false;
    private static final boolean tracePidInfo = false;

    private final String[] driveMotorNames = {
        RobotParams.LFDRIVE_MOTOR_NAME, RobotParams.RFDRIVE_MOTOR_NAME,
        RobotParams.LBDRIVE_MOTOR_NAME, RobotParams.RBDRIVE_MOTOR_NAME};
    private final int[] driveMotorIds = {
        RobotParams.CANID_LFDRIVE_MOTOR, RobotParams.CANID_RFDRIVE_MOTOR,
        RobotParams.CANID_LBDRIVE_MOTOR, RobotParams.CANID_RBDRIVE_MOTOR};
    private final boolean[] driveMotorInverted = {
        RobotParams.LFDRIVE_MOTOR_INVERTED, RobotParams.RFDRIVE_MOTOR_INVERTED,
        RobotParams.LBDRIVE_MOTOR_INVERTED, RobotParams.RBDRIVE_MOTOR_INVERTED};
    private final String[] steerEncoderNames = {
        RobotParams.LFSTEER_ENCODER_NAME, RobotParams.RFSTEER_ENCODER_NAME,
        RobotParams.LBSTEER_ENCODER_NAME, RobotParams.RBSTEER_ENCODER_NAME};
    private final int[] steerEncoderIds = {
        RobotParams.AIN_LFSTEER_ENCODER, RobotParams.AIN_RFSTEER_ENCODER,
        RobotParams.AIN_LBSTEER_ENCODER, RobotParams.AIN_RBSTEER_ENCODER};
    private final boolean[] steerEncoderInverted = {
        RobotParams.LFSTEER_ENCODER_INVERTED, RobotParams.RFSTEER_ENCODER_INVERTED,
        RobotParams.LBSTEER_ENCODER_INVERTED, RobotParams.RBSTEER_ENCODER_INVERTED};
    private final String[] steerMotorNames = {
        RobotParams.LFSTEER_MOTOR_NAME, RobotParams.RFSTEER_MOTOR_NAME,
        RobotParams.LBSTEER_MOTOR_NAME, RobotParams.RBSTEER_MOTOR_NAME};
    private final int[] steerMotorIds = {
        RobotParams.CANID_LFSTEER_MOTOR, RobotParams.CANID_RFSTEER_MOTOR,
        RobotParams.CANID_LBSTEER_MOTOR, RobotParams.CANID_RBSTEER_MOTOR};
    private final boolean[] steerMotorInverted = {
        RobotParams.LFSTEER_INVERTED, RobotParams.RFSTEER_INVERTED,
        RobotParams.LBSTEER_INVERTED, RobotParams.RBSTEER_INVERTED};
    private final String[] swerveModuleNames = {
        RobotParams.LFSWERVE_MODULE_NAME, RobotParams.RFSWERVE_MODULE_NAME,
        RobotParams.LBSWERVE_MODULE_NAME, RobotParams.RBSWERVE_MODULE_NAME};
    //
    // Swerve steering motors and modules.
    //
    public final TrcEncoder[] steerEncoders;
    public final TrcMotor[] steerMotors;
    public final TrcSwerveModule[] swerveModules;
    public int steerZeroCalibrationCount = 0;
    private String antiDefenseOwner = null;
    private boolean steerEncodersSynced = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public SwerveDrive(Robot robot)
    {
        super(robot);

        driveMotors = createMotors(MotorType.CAN_FALCON, false, driveMotorNames, driveMotorIds, driveMotorInverted);
        steerEncoders = createSteerEncoders(
            steerEncoderNames, steerEncoderIds, steerEncoderInverted, readSteeringCalibrationData());
        steerMotors = createMotors(MotorType.CAN_FALCON, false, steerMotorNames, steerMotorIds, steerMotorInverted);
        swerveModules = createSwerveModules(swerveModuleNames, driveMotors, steerMotors, steerEncoders);
        driveBase = new TrcSwerveDriveBase(
            swerveModules[INDEX_LEFT_FRONT], swerveModules[INDEX_LEFT_BACK],
            swerveModules[INDEX_RIGHT_FRONT], swerveModules[INDEX_RIGHT_BACK], gyro,
            RobotParams.ROBOT_WHEELBASE_WIDTH, RobotParams.ROBOT_WHEELBASE_LENGTH);
        driveBase.setOdometryScales(RobotParams.SWERVE_INCHES_PER_COUNT, RobotParams.SWERVE_INCHES_PER_COUNT);

        if (RobotParams.Preferences.useAntiTipping)
        {
            driveBase.enableAntiTipping(
                new TrcPidController.PidParameters(
                    RobotParams.X_TIPPING_KP, RobotParams.X_TIPPING_KI, RobotParams.X_TIPPING_KD,
                    RobotParams.X_TIPPING_TOLERANCE, this::getGyroRoll),
                new TrcPidController.PidParameters(
                    RobotParams.Y_TIPPING_KP, RobotParams.Y_TIPPING_KI, RobotParams.Y_TIPPING_KD,
                    RobotParams.Y_TIPPING_TOLERANCE, this::getGyroPitch));
        }
        // if (RobotParams.Preferences.useExternalOdometry)
        // {
        //     //
        //     // Create the external odometry device that uses the right back encoder port as the X odometry and
        //     // the left and right front encoder ports as the Y1 and Y2 odometry. Gyro will serve as the angle
        //     // odometry.
        //     //
        //     TrcDriveBaseOdometry driveBaseOdometry = new TrcDriveBaseOdometry(
        //         new TrcDriveBaseOdometry.AxisSensor(rbDriveMotor, RobotParams.X_ODOMETRY_WHEEL_OFFSET),
        //         new TrcDriveBaseOdometry.AxisSensor[] {
        //             new TrcDriveBaseOdometry.AxisSensor(lfDriveMotor, RobotParams.Y_LEFT_ODOMETRY_WHEEL_OFFSET),
        //             new TrcDriveBaseOdometry.AxisSensor(rfDriveMotor, RobotParams.Y_RIGHT_ODOMETRY_WHEEL_OFFSET)},
        //         gyro);
        //     //
        //     // Set the drive base to use the external odometry device overriding the built-in one.
        //     //
        //     driveBase.setDriveBaseOdometry(driveBaseOdometry);
        //     driveBase.setOdometryScales(RobotParams.ODWHEEL_X_INCHES_PER_COUNT, RobotParams.ODWHEEL_Y_INCHES_PER_COUNT);
        // }
        // else
        // {
        //     driveBase.setOdometryScales(RobotParams.SWERVE_INCHES_PER_COUNT);
        // }

        if (robot.pdp != null)
        {
            robot.pdp.registerEnergyUsed(
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LFDRIVE_MOTOR, driveMotorNames[INDEX_LEFT_FRONT]),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LBDRIVE_MOTOR, driveMotorNames[INDEX_LEFT_BACK]),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RFDRIVE_MOTOR, driveMotorNames[INDEX_RIGHT_FRONT]),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RBDRIVE_MOTOR, driveMotorNames[INDEX_RIGHT_BACK]),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LFSTEER_MOTOR, steerMotorNames[INDEX_LEFT_FRONT]),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LBSTEER_MOTOR, steerMotorNames[INDEX_LEFT_BACK]),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RFSTEER_MOTOR, steerMotorNames[INDEX_RIGHT_FRONT]),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RBSTEER_MOTOR, steerMotorNames[INDEX_RIGHT_BACK]));
        }
        //
        // Create and initialize PID controllers.
        //
        // PID Parameters for X and Y are the same for Swerve Drive.
        xPosPidCoeff = yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.SWERVE_KP, RobotParams.SWERVE_KI, RobotParams.SWERVE_KD, RobotParams.SWERVE_KF, RobotParams.SWERVE_IZONE);
        turnPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.GYRO_TURN_KP, RobotParams.GYRO_TURN_KI, RobotParams.GYRO_TURN_KD, RobotParams.GYRO_TURN_KF,
            RobotParams.GYRO_TURN_IZONE);
        velPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ROBOT_VEL_KP, RobotParams.ROBOT_VEL_KI, RobotParams.ROBOT_VEL_KD, RobotParams.ROBOT_VEL_KF);

        pidDrive = new TrcPidDrive(
            "pidDrive", driveBase,
            new TrcPidController.PidParameters(xPosPidCoeff, RobotParams.SWERVE_TOLERANCE, driveBase::getXPosition),
            new TrcPidController.PidParameters(yPosPidCoeff, RobotParams.SWERVE_TOLERANCE, driveBase::getYPosition),
            new TrcPidController.PidParameters(turnPidCoeff, RobotParams.GYRO_TURN_TOLERANCE, driveBase::getHeading));

        TrcPidController xPidCtrl = pidDrive.getXPidCtrl();
        xPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_XPID_POWER);
        xPidCtrl.setRampRate(RobotParams.DRIVE_MAX_XPID_RAMP_RATE);

        TrcPidController yPidCtrl = pidDrive.getYPidCtrl();
        yPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_YPID_POWER);
        yPidCtrl.setRampRate(RobotParams.DRIVE_MAX_YPID_RAMP_RATE);

        TrcPidController turnPidCtrl = pidDrive.getTurnPidCtrl();
        turnPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_TURNPID_POWER);
        turnPidCtrl.setRampRate(RobotParams.DRIVE_MAX_TURNPID_RAMP_RATE);
        turnPidCtrl.setAbsoluteSetPoint(true);

        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setMsgTracer(robot.globalTracer, logPoseEvents, tracePidInfo);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase, RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE,
            RobotParams.PPD_TURN_TOLERANCE, xPosPidCoeff, yPosPidCoeff, turnPidCoeff, velPidCoeff);
        purePursuitDrive.setMoveOutputLimit(RobotParams.PPD_MOVE_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setRotOutputLimit(RobotParams.PPD_ROT_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setMsgTracer(robot.globalTracer, logPoseEvents, tracePidInfo);
    }   //SwerveDrive

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
        final String funcName = "createSteerEncoder";
        TrcEncoder[] encoders = null;

        if (RobotParams.Preferences.useSteeringCANCoder)
        {
            ErrorCode errCode;
            encoders = new FrcCANCoder[names.length];
            for (int i = 0; i < names.length; i++)
            {
                FrcCANCoder canCoder = new FrcCANCoder(names[i], encoderIds[i]);
                errCode = canCoder.configFactoryDefault(30);
                if (errCode != ErrorCode.OK)
                {
                    robot.globalTracer.traceWarn(
                        funcName, "%s: CANcoder.configFactoryDefault failed (code=%s).",
                        names[i], errCode);
                }
                errCode = canCoder.configFeedbackCoefficient(1.0, "cpr", SensorTimeBase.PerSecond, 30);
                if (errCode != ErrorCode.OK)
                {
                    robot.globalTracer.traceWarn(
                        funcName, "%s: CANcoder.configFeedbackCoefficient failed (code=%s).",
                        names[i], errCode);
                }
                // Configure the encoder to initialize to absolute position value at boot.
                errCode = canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 30);
                if (errCode != ErrorCode.OK)
                {
                    robot.globalTracer.traceWarn(
                        funcName, "%s: CANcoder.configSensorInitializationStrategy failed (code=%s).",
                        names[i], errCode);
                }
                // Slow down the status frame rate to reduce CAN traffic.
                errCode = canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 30);
                if (errCode != ErrorCode.OK)
                {
                    robot.globalTracer.traceWarn(
                        funcName, "%s: CANcoder.setStatusFramePeriod failed (code=%s).",
                        names[i], errCode);
                }
                // Configure the sensor direction to match the steering motor direction.
                canCoder.setInverted(inverted[i]);
                // Normalize encoder to the range of 0 to 1.0 for a revolution (revolution per count).
                canCoder.setScaleAndOffset(1.0 / RobotParams.CANCODER_CPR, steerZeros[i]);
                encoders[i] = canCoder;
            }
        }
        else if (RobotParams.Preferences.useSteeringAnalogEncoder)
        {
            encoders = new FrcAnalogEncoder[names.length];
            for (int i = 0; i < names.length; i++)
            {
                FrcAnalogEncoder analogEncoder = new FrcAnalogEncoder(names[i], encoderIds[i]);
                analogEncoder.setInverted(inverted[i]);
                // Analog Encoder is already normalized to the range of 0 to 1.0 for a revolution (revolution per count).
                analogEncoder.setScaleAndOffset(1.0, steerZeros[i]);
                encoders[i] = analogEncoder;
            }
        }
        else
        {
            throw new IllegalArgumentException("Must enable either useCANCoder or useAnalogEncoder.");
        }

        return encoders;
    }   //createSteerEncoders

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
        final String funcName = "createSwerveModules";
        TrcSwerveModule[] modules = new TrcSwerveModule[names.length];

        for (int i = 0; i < names.length; i++)
        {
            // getPosition returns a value in the range of 0 to 1.0 of one revolution.
            double encoderPos = steerEncoders[i].getPosition();

            encoderPos *= RobotParams.STEER_MOTOR_CPR;
            ErrorCode errCode = ((FrcCANFalcon) steerMotors[i]).motor.setSelectedSensorPosition(encoderPos, 0, 30);
            if (errCode != ErrorCode.OK)
            {
                robot.globalTracer.traceWarn(
                    funcName, "%s: Falcon.setSelectedSensorPosition failed (code=%s, pos=%.0f).",
                    names[i], errCode, encoderPos);
            }
            // steerMotors[i].setControlMode(TalonFXControlMode.MotionMagic);

            // We have already synchronized the Falcon internal encoder with the zero adjusted absolute encoder, so
            // Falcon servo does not need to compensate for zero position.
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
        double lfSteerAbsEnc = steerEncoders[INDEX_LEFT_FRONT].getPosition()*360.0;
        // if (lfSteerAbsEnc > 90.0) lfSteerAbsEnc = 180.0 - lfSteerAbsEnc;
        double rfSteerAbsEnc = steerEncoders[INDEX_RIGHT_FRONT].getPosition()*360.0;
        // if (rfSteerAbsEnc > 90.0) rfSteerAbsEnc = 180.0 - rfSteerAbsEnc;
        double lbSteerAbsEnc = steerEncoders[INDEX_LEFT_BACK].getPosition()*360.0;
        // if (lbSteerAbsEnc > 90.0) lbSteerAbsEnc = 180.0 - lbSteerAbsEnc;
        double rbSteerAbsEnc = steerEncoders[INDEX_RIGHT_BACK].getPosition()*360.0;
        // if (rbSteerAbsEnc > 90.0) rbSteerAbsEnc = 180.0 - rbSteerAbsEnc;
        double lfSteerEnc = (steerMotors[INDEX_LEFT_FRONT].getMotorPosition() % RobotParams.STEER_MOTOR_CPR) /
                            RobotParams.STEER_MOTOR_CPR * 360.0;
        double rfSteerEnc = (steerMotors[INDEX_RIGHT_FRONT].getMotorPosition() % RobotParams.STEER_MOTOR_CPR) /
                            RobotParams.STEER_MOTOR_CPR * 360.0;
        double lbSteerEnc = (steerMotors[INDEX_LEFT_BACK].getMotorPosition() % RobotParams.STEER_MOTOR_CPR) /
                            RobotParams.STEER_MOTOR_CPR * 360.0;
        double rbSteerEnc = (steerMotors[INDEX_RIGHT_BACK].getMotorPosition() % RobotParams.STEER_MOTOR_CPR) /
                            RobotParams.STEER_MOTOR_CPR * 360.0;

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
        final String funcName = "syncSteerEncoders";
        final double encErrThreshold = 20.0;
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
                        robot.globalTracer.traceInfo(
                            funcName, "[%.3f] steerEncPos[%d]=%.0f", TrcTimer.getModeElapsedTime(), i, steerPos);
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
                    double encoderPos = steerEncoders[i].getPosition() * RobotParams.STEER_MOTOR_CPR;
                    ((FrcCANFalcon) steerMotors[i]).motor.setSelectedSensorPosition(encoderPos, 0, 0);
                    robot.globalTracer.traceInfo(
                        funcName, "[%.3f] syncSteerEncPos[%d]=%.0f", TrcTimer.getModeElapsedTime(), i, encoderPos);
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
            syncSteerEncoders(false);
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
     *
     * @param steerZeros specifies the steer zero calibration data array to be initialized.
     */
    public void startSteeringCalibration(double[] steerZeros)
    {
        steerZeroCalibrationCount = 0;
        Arrays.fill(steerZeros, 0.0);
    }   //startSteeringCalibration

    /**
     * This method stops the steering calibration and saves the calibration data to a file.
     *
     * @param steerZeros specifies the steer zero calibration data array to be saved.
     */
    public void stopSteeringCalibration(double[] steerZeros)
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
     *
     * @param steerZeros specifies the steer zero calibration data array to be updated.
     */
    public void runSteeringCalibration(double[] steerZeros)
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
        final String funcName = "saveSteeringCalibrationData";

        try (PrintStream out = new PrintStream(new FileOutputStream(
            RobotParams.TEAM_FOLDER_PATH + "/" + RobotParams.STEER_ZERO_CAL_FILE)))
        {
            for (int i = 0; i < steerMotorNames.length; i++)
            {
                out.printf("%s: %f\n", steerMotorNames[i], steerZeros[i]);
            }
            out.close();
            TrcDbgTrace.getGlobalTracer().traceInfo(
                funcName, "SteeringCalibrationData%s=%s",
                Arrays.toString(steerMotorNames), Arrays.toString(steerZeros));
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
        final String funcName = "readSteeringCalibrationData";
        TrcDbgTrace tracer = TrcDbgTrace.getGlobalTracer();
        String line = null;

        try (Scanner in = new Scanner(new FileReader(
            RobotParams.TEAM_FOLDER_PATH + "/" + RobotParams.STEER_ZERO_CAL_FILE)))
        {
            double[] steerZeros = new double[steerMotors.length];

            for (int i = 0; i < steerMotors.length; i++)
            {
                line = in.nextLine();
                int colonPos = line.indexOf(':');
                String name = colonPos == -1? null: line.substring(0, colonPos);

                if (name == null || !name.equals(steerMotorNames[i]))
                {
                    throw new RuntimeException("Invalid steer motor name in line " + line);
                }

                steerZeros[i] = Double.parseDouble(line.substring(colonPos + 1));
            }
            tracer.traceInfo(
                funcName, "SteeringCalibrationData%s=%s",
                Arrays.toString(steerMotorNames), Arrays.toString(steerZeros));

            return steerZeros;
        }
        catch (FileNotFoundException e)
        {
            tracer.traceWarn(funcName, "Steering calibration data file not found, using built-in defaults.");
            return RobotParams.STEER_ZEROS;
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
     * This method checks if anti-defense mode is enabled.
     *
     * @return true if anti-defense mode is enabled, false if disabled.
     */
    public boolean isAntiDefenseEnabled()
    {
        return ((TrcSwerveDriveBase) driveBase).isAntiDefenseEnabled();
    }   //isAntiDefenseEnabled

    /**
     * This method enables/disables the anti-defense mode where it puts all swerve wheels into an X-formation.
     * By doing so, it is very difficult for others to push us around.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param enabled   specifies true to enable anti-defense mode, false to disable.
     */
    public void setAntiDefenseEnabled(String owner, boolean enabled)
    {
        boolean requireOwnership = owner != null && enabled && !driveBase.hasOwnership(owner);

        if (requireOwnership && driveBase.acquireExclusiveAccess(owner))
        {
            antiDefenseOwner = owner;
        }

        if (!requireOwnership || antiDefenseOwner != null)
        {
            ((TrcSwerveDriveBase) driveBase).setAntiDefenseEnabled(owner, enabled);
            if (antiDefenseOwner != null)
            {
                driveBase.releaseExclusiveAccess(antiDefenseOwner);
                antiDefenseOwner = null;
            }
        }
    }   //setAntiDefenseEnabled

}   //class SwerveDrive
