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

import TrcCommonLib.trclib.TrcMecanumDriveBase;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcFrcLib.frclib.FrcPdp;
import team492.Robot;
import team492.RobotParams;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class MecanumDrive extends RobotDrive
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

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public MecanumDrive(Robot robot)
    {
        super(robot);

        driveMotors = createMotors(MotorType.CAN_FALCON, false, driveMotorNames, driveMotorIds, driveMotorInverted);
        driveBase = new TrcMecanumDriveBase(
            driveMotors[INDEX_LEFT_FRONT], driveMotors[INDEX_LEFT_BACK],
            driveMotors[INDEX_RIGHT_FRONT], driveMotors[INDEX_RIGHT_BACK], gyro);
        driveBase.setOdometryScales(RobotParams.MECANUM_X_INCHES_PER_COUNT, RobotParams.MECANUM_Y_INCHES_PER_COUNT);

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
        //     driveBase.setOdometryScales(RobotParams.MECANUM_X_INCHES_PER_COUNT, RobotParams.MECANUM_Y_INCHES_PER_COUNT);
        // }

        if (robot.pdp != null)
        {
            robot.pdp.registerEnergyUsed(
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LFDRIVE_MOTOR, driveMotorNames[INDEX_LEFT_FRONT]),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LBDRIVE_MOTOR, driveMotorNames[INDEX_LEFT_BACK]),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RFDRIVE_MOTOR, driveMotorNames[INDEX_RIGHT_FRONT]),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RBDRIVE_MOTOR, driveMotorNames[INDEX_RIGHT_BACK]));
        }

        //
        // Create and initialize PID controllers.
        //
        xPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.MECANUM_X_KP, RobotParams.MECANUM_X_KI, RobotParams.MECANUM_X_KD, RobotParams.MECANUM_X_KF);
        yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.MECANUM_Y_KP, RobotParams.MECANUM_Y_KI, RobotParams.MECANUM_Y_KD, RobotParams.MECANUM_Y_KF);
        turnPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.GYRO_TURN_KP, RobotParams.GYRO_TURN_KI, RobotParams.GYRO_TURN_KD, RobotParams.GYRO_TURN_KF);
        velPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ROBOT_VEL_KP, RobotParams.ROBOT_VEL_KI, RobotParams.ROBOT_VEL_KD, RobotParams.ROBOT_VEL_KF);

        pidDrive = new TrcPidDrive(
            "pidDrive", driveBase,
            new TrcPidController.PidParameters(xPosPidCoeff, RobotParams.MECANUM_X_TOLERANCE, driveBase::getXPosition),
            new TrcPidController.PidParameters(yPosPidCoeff, RobotParams.MECANUM_Y_TOLERANCE, driveBase::getYPosition),
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
    }   //MecanumDrive

}   //class MecanumDrive
