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

import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcPurePursuitDrive;
import TrcCommonLib.trclib.TrcSimpleDriveBase;
import TrcCommonLib.trclib.TrcDbgTrace.MsgLevel;
import TrcFrcLib.frclib.FrcPdp;
import team492.Robot;
import team492.RobotParams;

/**
 * This class creates the RobotDrive subsystem that consists of wheel motors and related objects for driving the
 * robot.
 */
public class DifferentialDrive extends RobotDrive
{
    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     * @param driveBaseParams specifies the drivebase parameters.
     */
    public DifferentialDrive(Robot robot, RobotParams.DifferentialDriveBase driveBaseParams)
    {
        super(robot);

        driveMotors = createMotors(
            MotorType.CanSparkMax, true, driveBaseParams.driveMotorNames, driveBaseParams.driveMotorIds,
            driveBaseParams.driveMotorInverted);
        driveBase = new TrcSimpleDriveBase(
            driveMotors[RobotDrive.INDEX_LEFT_FRONT], driveMotors[RobotDrive.INDEX_RIGHT_FRONT], gyro);
        driveBase.setOdometryScales(driveBaseParams.DRIVE_INCHES_PER_COUNT);

        if (robot.pdp != null)
        {
            robot.pdp.registerEnergyUsed(
                new FrcPdp.Channel(
                    RobotParams.PDP_CHANNEL_LFDRIVE_MOTOR,
                    driveBaseParams.driveMotorNames[RobotDrive.INDEX_LEFT_FRONT]),
                new FrcPdp.Channel(
                    RobotParams.PDP_CHANNEL_RFDRIVE_MOTOR,
                    driveBaseParams.driveMotorNames[RobotDrive.INDEX_RIGHT_FRONT]));
        }
        //
        // Create and initialize PID controllers.
        //
        xPosPidCoeff = null;
        yPosPidCoeff = new TrcPidController.PidCoefficients(
            driveBaseParams.DRIVE_KP, driveBaseParams.DRIVE_KI, driveBaseParams.DRIVE_KD, driveBaseParams.DRIVE_KF);
        turnPidCoeff = new TrcPidController.PidCoefficients(
            driveBaseParams.TURN_KP, driveBaseParams.TURN_KI, driveBaseParams.TURN_KD, driveBaseParams.TURN_KF);
        velPidCoeff = new TrcPidController.PidCoefficients(
            driveBaseParams.ROBOT_VEL_KP, driveBaseParams.ROBOT_VEL_KI, driveBaseParams.ROBOT_VEL_KD,
            driveBaseParams.ROBOT_VEL_KF);

        pidDrive = new TrcPidDrive(
            "pidDrive", driveBase,
            yPosPidCoeff, driveBaseParams.DRIVE_TOLERANCE, driveBase::getYPosition,
            turnPidCoeff, driveBaseParams.TURN_TOLERANCE, driveBase::getHeading);

        TrcPidController yPidCtrl = pidDrive.getYPidCtrl();
        yPidCtrl.setOutputLimit(driveBaseParams.DRIVE_MAX_PID_POWER);
        yPidCtrl.setRampRate(driveBaseParams.DRIVE_MAX_PID_RAMP_RATE);

        TrcPidController turnPidCtrl = pidDrive.getTurnPidCtrl();
        turnPidCtrl.setOutputLimit(driveBaseParams.TURN_MAX_PID_POWER);
        turnPidCtrl.setRampRate(driveBaseParams.TURN_MAX_PID_RAMP_RATE);
        turnPidCtrl.setAbsoluteSetPoint(true);

        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setStallDetectionEnabled(true);
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setTraceLevel(MsgLevel.INFO, false, false, false);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase,
            driveBaseParams.PPD_FOLLOWING_DISTANCE, driveBaseParams.PPD_POS_TOLERANCE,
            driveBaseParams.PPD_TURN_TOLERANCE, xPosPidCoeff, yPosPidCoeff, turnPidCoeff, velPidCoeff);
        purePursuitDrive.setStallDetectionEnabled(true);
        purePursuitDrive.setMoveOutputLimit(driveBaseParams.PPD_MOVE_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setRotOutputLimit(driveBaseParams.PPD_ROT_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setTraceLevel(MsgLevel.INFO, false, false, false);
    }   //DifferentialDrive

}   //class DifferentialDrive
