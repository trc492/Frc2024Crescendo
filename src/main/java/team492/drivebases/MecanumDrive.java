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
import TrcCommonLib.trclib.TrcDbgTrace.MsgLevel;
import TrcFrcLib.frclib.FrcPdp;
import team492.Robot;
import team492.RobotParams;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class MecanumDrive extends RobotDrive
{
    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public MecanumDrive(Robot robot)
    {
        super(robot);

        driveMotors = createMotors(
            MotorType.CanFalcon, false, RobotParams.MecanumDriveBase.driveMotorNames,
            RobotParams.MecanumDriveBase.driveMotorIds, RobotParams.MecanumDriveBase.driveMotorInverted);
        driveBase = new TrcMecanumDriveBase(
            driveMotors[RobotDrive.INDEX_LEFT_FRONT], driveMotors[RobotDrive.INDEX_LEFT_BACK],
            driveMotors[RobotDrive.INDEX_RIGHT_FRONT], driveMotors[RobotDrive.INDEX_RIGHT_BACK], gyro);
        driveBase.setOdometryScales(
            RobotParams.MecanumDriveBase.DRIVE_X_INCHES_PER_COUNT, RobotParams.MecanumDriveBase.DRIVE_Y_INCHES_PER_COUNT);

        if (robot.pdp != null)
        {
            robot.pdp.registerEnergyUsed(
                new FrcPdp.Channel(
                    RobotParams.PDP_CHANNEL_LFDRIVE_MOTOR,
                    RobotParams.MecanumDriveBase.driveMotorNames[RobotDrive.INDEX_LEFT_FRONT]),
                new FrcPdp.Channel(
                    RobotParams.PDP_CHANNEL_LBDRIVE_MOTOR,
                    RobotParams.MecanumDriveBase.driveMotorNames[RobotDrive.INDEX_LEFT_BACK]),
                new FrcPdp.Channel(
                    RobotParams.PDP_CHANNEL_RFDRIVE_MOTOR,
                    RobotParams.MecanumDriveBase.driveMotorNames[RobotDrive.INDEX_RIGHT_FRONT]),
                new FrcPdp.Channel(
                    RobotParams.PDP_CHANNEL_RBDRIVE_MOTOR,
                    RobotParams.MecanumDriveBase.driveMotorNames[RobotDrive.INDEX_RIGHT_BACK]));
        }
        //
        // Create and initialize PID controllers.
        //
        xPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.MecanumDriveBase.DRIVE_X_KP, RobotParams.MecanumDriveBase.DRIVE_X_KI,
            RobotParams.MecanumDriveBase.DRIVE_X_KD, RobotParams.MecanumDriveBase.DRIVE_X_KF);
        yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.MecanumDriveBase.DRIVE_Y_KP, RobotParams.MecanumDriveBase.DRIVE_Y_KI,
            RobotParams.MecanumDriveBase.DRIVE_Y_KD, RobotParams.MecanumDriveBase.DRIVE_Y_KF);
        turnPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.MecanumDriveBase.TURN_KP, RobotParams.MecanumDriveBase.TURN_KI,
            RobotParams.MecanumDriveBase.TURN_KD, RobotParams.MecanumDriveBase.TURN_KF);
        velPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.MecanumDriveBase.ROBOT_VEL_KP, RobotParams.MecanumDriveBase.ROBOT_VEL_KI,
            RobotParams.MecanumDriveBase.ROBOT_VEL_KD, RobotParams.MecanumDriveBase.ROBOT_VEL_KF);

        pidDrive = new TrcPidDrive(
            "pidDrive", driveBase,
            xPosPidCoeff, RobotParams.MecanumDriveBase.DRIVE_X_TOLERANCE, driveBase::getXPosition,
            yPosPidCoeff, RobotParams.MecanumDriveBase.DRIVE_Y_TOLERANCE, driveBase::getYPosition,
            turnPidCoeff, RobotParams.MecanumDriveBase.TURN_TOLERANCE, driveBase::getHeading);

        TrcPidController xPidCtrl = pidDrive.getXPidCtrl();
        xPidCtrl.setOutputLimit(RobotParams.MecanumDriveBase.DRIVE_MAX_XPID_POWER);
        xPidCtrl.setRampRate(RobotParams.MecanumDriveBase.DRIVE_MAX_XPID_RAMP_RATE);

        TrcPidController yPidCtrl = pidDrive.getYPidCtrl();
        yPidCtrl.setOutputLimit(RobotParams.MecanumDriveBase.DRIVE_MAX_YPID_POWER);
        yPidCtrl.setRampRate(RobotParams.MecanumDriveBase.DRIVE_MAX_YPID_RAMP_RATE);

        TrcPidController turnPidCtrl = pidDrive.getTurnPidCtrl();
        turnPidCtrl.setOutputLimit(RobotParams.MecanumDriveBase.DRIVE_MAX_TURNPID_POWER);
        turnPidCtrl.setRampRate(RobotParams.MecanumDriveBase.DRIVE_MAX_TURNPID_RAMP_RATE);
        turnPidCtrl.setAbsoluteSetPoint(true);

        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setStallDetectionEnabled(true);
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setTraceLevel(MsgLevel.INFO, false, false, false);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase,
            RobotParams.MecanumDriveBase.PPD_FOLLOWING_DISTANCE, RobotParams.MecanumDriveBase.PPD_POS_TOLERANCE,
            RobotParams.MecanumDriveBase.PPD_TURN_TOLERANCE, xPosPidCoeff, yPosPidCoeff, turnPidCoeff, velPidCoeff);
        purePursuitDrive.setStallDetectionEnabled(true);
        purePursuitDrive.setMoveOutputLimit(RobotParams.MecanumDriveBase.PPD_MOVE_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setRotOutputLimit(RobotParams.MecanumDriveBase.PPD_ROT_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setTraceLevel(MsgLevel.INFO, false, false, false);
    }   //MecanumDrive

}   //class MecanumDrive
