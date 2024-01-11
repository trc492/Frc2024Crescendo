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
    private final String[] driveMotorNames = {
        RobotParams.LFDRIVE_MOTOR_NAME, RobotParams.RFDRIVE_MOTOR_NAME};
    private final int[] driveMotorIds = {
        RobotParams.CANID_LFDRIVE_MOTOR, RobotParams.CANID_RFDRIVE_MOTOR};
    private final boolean[] driveMotorInverted = {
        RobotParams.LFDRIVE_MOTOR_INVERTED, RobotParams.RFDRIVE_MOTOR_INVERTED};

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public DifferentialDrive(Robot robot)
    {
        super(robot);

        driveMotors = createMotors(MotorType.CanSparkMax, true, driveMotorNames, driveMotorIds, driveMotorInverted);
        driveBase = new TrcSimpleDriveBase(driveMotors[INDEX_LEFT_FRONT], driveMotors[INDEX_RIGHT_FRONT], gyro);
        driveBase.setOdometryScales(RobotParams.WCD_INCHES_PER_ENCODER_UNIT);

        if (robot.pdp != null)
        {
            robot.pdp.registerEnergyUsed(
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_LFDRIVE_MOTOR, driveMotorNames[INDEX_LEFT_FRONT]),
                new FrcPdp.Channel(RobotParams.PDP_CHANNEL_RFDRIVE_MOTOR, driveMotorNames[INDEX_RIGHT_FRONT]));
        }
        //
        // Create and initialize PID controllers.
        //
        xPosPidCoeff = null;
        yPosPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.WCD_KP, RobotParams.WCD_KI, RobotParams.WCD_KD, RobotParams.WCD_KF);
        turnPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.GYRO_TURN_KP, RobotParams.GYRO_TURN_KI, RobotParams.GYRO_TURN_KD, RobotParams.GYRO_TURN_KF);
        velPidCoeff = new TrcPidController.PidCoefficients(
            RobotParams.ROBOT_VEL_KP, RobotParams.ROBOT_VEL_KI, RobotParams.ROBOT_VEL_KD, RobotParams.ROBOT_VEL_KF);

        pidDrive = new TrcPidDrive(
            "pidDrive", driveBase,
            yPosPidCoeff, RobotParams.WCD_TOLERANCE, driveBase::getYPosition,
            turnPidCoeff, RobotParams.GYRO_TURN_TOLERANCE, driveBase::getHeading);

        TrcPidController yPidCtrl = pidDrive.getYPidCtrl();
        yPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_YPID_POWER);
        yPidCtrl.setRampRate(RobotParams.DRIVE_MAX_YPID_RAMP_RATE);

        TrcPidController turnPidCtrl = pidDrive.getTurnPidCtrl();
        turnPidCtrl.setOutputLimit(RobotParams.DRIVE_MAX_TURNPID_POWER);
        turnPidCtrl.setRampRate(RobotParams.DRIVE_MAX_TURNPID_RAMP_RATE);
        turnPidCtrl.setAbsoluteSetPoint(true);

        // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
        // of the absolute target position.
        pidDrive.setStallDetectionEnabled(true);
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setTraceLevel(MsgLevel.INFO, false, false, false);

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase, RobotParams.PPD_FOLLOWING_DISTANCE, RobotParams.PPD_POS_TOLERANCE,
            RobotParams.PPD_TURN_TOLERANCE, xPosPidCoeff, yPosPidCoeff, turnPidCoeff, velPidCoeff);
        purePursuitDrive.setStallDetectionEnabled(true);
        purePursuitDrive.setMoveOutputLimit(RobotParams.PPD_MOVE_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setRotOutputLimit(RobotParams.PPD_ROT_DEF_OUTPUT_LIMIT);
        purePursuitDrive.setFastModeEnabled(true);
        purePursuitDrive.setTraceLevel(MsgLevel.INFO, false, false, false);
    }   //DifferentialDrive

}   //class DifferentialDrive
