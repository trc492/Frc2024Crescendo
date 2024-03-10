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

package team492.autocommands;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team492.FrcAuto;
import team492.Robot;
import team492.RobotParams;
import team492.FrcAuto.AutoChoices;
import team492.FrcAuto.AutoStartPos;
import team492.autotasks.TaskAutoScoreNote.TargetType;

/**
 * This class implements an autonomous strategy.
 */
public class CmdAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdAuto.class.getSimpleName();

    private enum State
    {
        START,
        DO_DELAY,
        DRIVE_TO_WING_NOTE,
        PICKUP_WING_NOTE,
        SCORE_WING_NOTE,
        PERFORM_END_ACTION,
        PICKUP_CENTERLINE_NOTE,
        DONE
    }   //enum State

    private final Robot robot;
    private final AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private Alliance alliance;
    private AutoStartPos startPos;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdAuto(Robot robot, AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdAuto

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        timer.cancel();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + sm.getNextState() + ")...");
        }
        else
        {
            robot.dashboard.displayPrintf(8, "State: " + state);

            switch (state)
            {
                case START:
                    // Set robot start field position and score preloaded Note.
                    robot.robotDrive.setFieldPosition(false);
                    alliance = FrcAuto.autoChoices.getAlliance();
                    startPos = FrcAuto.autoChoices.getStartPos();

                    TargetType targetType;
                    boolean shootInPlace;
                    if (startPos == AutoStartPos.AMP)
                    {
                        targetType = TargetType.Amp;
                        shootInPlace = false;
                    }
                    else
                    {
                        targetType = TargetType.Speaker;
                        shootInPlace = true;
                    } 

                    robot.autoScoreNote.autoAssistScore(targetType, true, shootInPlace, event);
                    sm.waitForSingleEvent(event, State.DO_DELAY);
                    break;

                case DO_DELAY:
                    // Do delay if there is one.
                    double startDelay = autoChoices.getStartDelay();
                    if (startDelay > 0.0)
                    {
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_WING_NOTE);
                    }
                    else
                    {
                        sm.setState(State.DRIVE_TO_WING_NOTE);
                    }
                    break;

                case DRIVE_TO_WING_NOTE:
                    if (FrcAuto.autoChoices.getScoreWingNote())
                    {
                        // For SW_AMP_SIDE or SW_SOURCE_SIDE, we need to get to the position where the camera can
                        // see the Note.
                        if(startPos == AutoStartPos.SW_SOURCE_SIDE || startPos == AutoStartPos.SW_AMP_SIDE)
                        {
                            TrcPose2D wingNotePose =
                                RobotParams.Game.wingNotePoses[0][startPos == AutoStartPos.SW_SOURCE_SIDE? 0: 2].clone();
                            wingNotePose.y -= 24.0; // TODO: Find Actual Distance
                            wingNotePose.angle = 180.0;
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                RobotParams.SwerveDriveBase.ROBOT_MAX_VELOCITY, RobotParams.SwerveDriveBase.ROBOT_MAX_ACCELERATION,
                                robot.adjustPoseByAlliance(wingNotePose, alliance));
                            sm.waitForSingleEvent(event, State.PICKUP_WING_NOTE);
                        }
                        else
                        {
                            sm.setState(State.PICKUP_WING_NOTE);
                        }
                    }
                    else
                    {
                        sm.setState(State.PERFORM_END_ACTION);
                    }
                    break;

                case PICKUP_WING_NOTE:
                    robot.autoPickupFromGround.autoAssistPickup(true, event);
                    sm.waitForSingleEvent(event, State.SCORE_WING_NOTE);
                    break;

                case SCORE_WING_NOTE:
                    robot.autoScoreNote.autoAssistScore(TargetType.Speaker, true, true, event);
                    sm.waitForSingleEvent(event, State.PERFORM_END_ACTION);
                    break;

                case PERFORM_END_ACTION:
                    TrcPose2D centerlineNotePose = RobotParams.Game.centerlineNotePoses[4].clone();
                    centerlineNotePose.y -= 72.0; // TODO: Find Actual Distance
                    centerlineNotePose.angle = 180.0;
                    if (startPos == AutoStartPos.SW_SOURCE_SIDE)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            RobotParams.SwerveDriveBase.ROBOT_MAX_VELOCITY, RobotParams.SwerveDriveBase.ROBOT_MAX_ACCELERATION,
                            robot.adjustPoseByAlliance(RobotParams.Game.WINGNOTE_BLUE_SW_SIDE, alliance),
                            robot.adjustPoseByAlliance(centerlineNotePose, alliance));
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            RobotParams.SwerveDriveBase.ROBOT_MAX_VELOCITY, RobotParams.SwerveDriveBase.ROBOT_MAX_ACCELERATION,
                            robot.adjustPoseByAlliance(centerlineNotePose, alliance));
                    }
                    sm.waitForSingleEvent(event, State.PICKUP_CENTERLINE_NOTE);
                    break;

                case PICKUP_CENTERLINE_NOTE:
                    FrcAuto.EndAction endAction = FrcAuto.autoChoices.getEndAction();
                    if (endAction == FrcAuto.EndAction.PARK_NEAR_CENTERLINE)
                    {
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        robot.autoPickupFromGround.autoAssistPickup(true, event);
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                default:
                case DONE:
                    // We are done.
                    cancel();
                    break;
            }

            robot.globalTracer.traceStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto
