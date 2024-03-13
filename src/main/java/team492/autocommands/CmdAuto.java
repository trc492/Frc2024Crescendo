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
import TrcFrcLib.frclib.FrcPhotonVision.DetectedObject;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team492.FrcAuto;
import team492.Robot;
import team492.RobotParams;
import team492.FrcAuto.AutoChoices;
import team492.FrcAuto.AutoStartPos;
import team492.FrcAuto.EndAction;
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
        TURN_TO_SPEAKER,
        SCORE_WING_NOTE,
        TURN_TO_WING_NOTES,
        DRIVE_TO_CENTER_LINE,
        PICKUP_CENTERLINE_NOTE,
        PARK,
        DONE
    }   //enum State

    private final Robot robot;
    private final AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent aprilTagEvent;
    private final TrcEvent noteEvent;
    private final TrcStateMachine<State> sm;
    private Alliance alliance;
    private AutoStartPos startPos;
    private boolean scoreWingNotes;
    private EndAction endAction;
    private boolean relocalize;
    private int numWingNotesScored = 0;
    private boolean aprilTagVisionEnabled = false;
    private boolean noteVisionEnabled = false;
    // private DetectedObject aprilTagObj = null;
    // private DetectedObject noteObj = null;

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
        aprilTagEvent = new TrcEvent(moduleName + ".aprilTagEvent");
        noteEvent = new TrcEvent(moduleName + ".noteEvent");
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

        // Must have vision to perform this autonomous.
        if (aprilTagVisionEnabled)
        {
            robot.photonVisionFront.getBestDetectedAprilTag(aprilTagEvent, new int[] {4, 8});
        }

        if (noteVisionEnabled)
        {
            DetectedObject noteObj = robot.photonVisionBack.getBestDetectedObject(noteEvent);
            if (noteObj != null)
            {
                if (noteObj.targetPose.y > 120.0 || Math.abs(noteObj.targetPose.angle) > 20.0)
                {
                    robot.globalTracer.traceInfo(
                        moduleName, "Vision found note too far or not turn enough at " + noteObj + ".");
                    noteEvent.clear();
                }
                else
                {
                    robot.globalTracer.traceInfo(moduleName, "Vision found note at " + noteObj + ".");
                }
            }
        }

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + sm.getNextState() + ")...");
        }
        else
        {
            TrcPose2D robotPose;
            TrcPose2D targetPose;

            robot.dashboard.displayPrintf(8, "State: " + state);
            switch (state)
            {
                case START:
                    // Set robot start field position and score preloaded Note.
                    robot.robotDrive.setFieldPosition(false);
                    alliance = FrcAuto.autoChoices.getAlliance();
                    startPos = FrcAuto.autoChoices.getStartPos();
                    scoreWingNotes = FrcAuto.autoChoices.getScoreWingNotes();
                    endAction = FrcAuto.autoChoices.getEndAction();
                    relocalize = FrcAuto.autoChoices.getRelocalize();

                    robot.shooter.aimShooter(
                        null, RobotParams.Shooter.shooterSpeakerCloseVelocity,
                        RobotParams.Shooter.tiltSpeakerCloseAngle, 0.0, event, 0.0, robot::shoot, 0.0);
                    sm.waitForSingleEvent(event, State.DO_DELAY);
                    break;

                case DO_DELAY:
                    // Do delay if there is one.
                    robot.shooter.aimShooter(
                        RobotParams.Shooter.wingNotePresetParams.shooterVelocity,
                        RobotParams.Shooter.wingNotePresetParams.tiltAngle, 0.0);

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
                    if (!scoreWingNotes)
                    {
                        sm.setState(State.DRIVE_TO_CENTER_LINE);
                    }
                    else
                    {
                        // For SW_AMP_SIDE or SW_SOURCE_SIDE, we need to get to the position where the camera can
                        // see the Note.
                        if(startPos == AutoStartPos.SW_SOURCE_SIDE || startPos == AutoStartPos.SW_AMP_SIDE)
                        {
                            robotPose = robot.robotDrive.driveBase.getFieldPosition();
                            TrcPose2D wingNotePose =
                                RobotParams.Game.wingNotePoses[0][startPos == AutoStartPos.SW_SOURCE_SIDE? 0: 2].clone();
                            wingNotePose.y -= 24.0;
                            wingNotePose.angle = 180.0;
                            robot.robotDrive.purePursuitDrive.start(
                                event, robotPose, false,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                                robot.adjustPoseByAlliance(wingNotePose, alliance));
                            sm.addEvent(event);
                            noteVisionEnabled = true;
                            sm.addEvent(noteEvent);
                            sm.waitForEvents(State.PICKUP_WING_NOTE, false);
                        }
                        else
                        {
                            sm.setState(State.PICKUP_WING_NOTE);
                        }
                    }
                    break;

                case PICKUP_WING_NOTE:
                    robot.robotDrive.purePursuitDrive.cancel();
                    noteVisionEnabled = false;
                    robot.autoPickupFromGround.autoAssistPickup(true, true, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_SPEAKER);
                    break;

                case TURN_TO_SPEAKER:
                    if (!robot.intake.hasObject())
                    {
                        // Failed to pick up a Note, probably vision failed, move to EndAction.
                        sm.setState(State.DRIVE_TO_CENTER_LINE);
                    }
                    else if (numWingNotesScored > 0 || startPos == AutoStartPos.AMP)
                    {
                        robotPose = robot.robotDrive.driveBase.getFieldPosition();
                        targetPose = robotPose.clone();
                        targetPose.angle = alliance == Alliance.Red? 0.0: 180.0;
                        robot.robotDrive.purePursuitDrive.start(
                            event, robotPose, false,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                            targetPose);
                        sm.addEvent(event);
                        aprilTagVisionEnabled = true;
                        sm.addEvent(aprilTagEvent);
                        sm.waitForEvents(State.SCORE_WING_NOTE, false);
                    }
                    else
                    {
                        sm.setState(State.SCORE_WING_NOTE);
                    }
                    break;

                case SCORE_WING_NOTE:
                    robot.robotDrive.purePursuitDrive.cancel();
                    aprilTagVisionEnabled = false;
                    robot.autoScoreNote.autoAssistScore(TargetType.Speaker, true, true, relocalize, event);
                    numWingNotesScored++;
                    robot.globalTracer.traceInfo(moduleName, "Scoring Wing Note " + numWingNotesScored);
                    sm.waitForSingleEvent(event, State.TURN_TO_WING_NOTES);
                    break;

                case TURN_TO_WING_NOTES:
                    if (startPos != AutoStartPos.SW_CENTER && numWingNotesScored < 3)
                    {
                        robotPose = robot.robotDrive.driveBase.getFieldPosition();
                        targetPose = robotPose.clone();
                        targetPose.angle =
                            startPos == AutoStartPos.SW_SOURCE_SIDE? 90.0: -90.0;
                        robot.robotDrive.purePursuitDrive.start(
                            event, robotPose, false,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                            targetPose);
                        sm.addEvent(event);
                        noteVisionEnabled = true;
                        sm.addEvent(noteEvent);
                        sm.waitForEvents(State.PICKUP_WING_NOTE, false);
                    }
                    else
                    {
                        sm.setState(State.DRIVE_TO_CENTER_LINE);
                    }
                    break;

                case DRIVE_TO_CENTER_LINE:
                    robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    int centerlineNoteIndex = Math.abs(robotPose.x) < RobotParams.Field.WIDTH / 2.0? 0: 4;
                    TrcPose2D centerlineNotePose =
                        RobotParams.Game.centerlineNotePoses[centerlineNoteIndex].clone();
                    centerlineNotePose.y -= 72.0;
                    centerlineNotePose.angle = 180.0;
                    robot.robotDrive.purePursuitDrive.start(
                        event, robotPose, false,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                        robot.adjustPoseByAlliance(centerlineNotePose, alliance));
                    sm.waitForSingleEvent(event, endAction == EndAction.PARK? State.DONE: State.PICKUP_CENTERLINE_NOTE);
                    break;

                case PICKUP_CENTERLINE_NOTE:
                    robot.autoPickupFromGround.autoAssistPickup(true, true, event);
                    sm.waitForSingleEvent(event, State.PARK);
                    break;

                case PARK:
                    // Make sure we are back on our side if we went over just a little.
                    robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    double halfFieldLength = RobotParams.Field.LENGTH / 2.0;
                    double halfRobotLength = RobotParams.Robot.LENGTH / 2.0;
                    double robotSafeY = 0.0;

                    if (alliance == Alliance.Red && robotPose.y - halfRobotLength < halfFieldLength)
                    {
                        robotSafeY = halfFieldLength + halfRobotLength + 24.0;
                    }
                    else if (alliance == Alliance.Blue && robotPose.y + halfRobotLength > halfFieldLength)
                    {
                        robotSafeY = halfFieldLength - halfRobotLength - 24.0;
                    }

                    if (robotSafeY != 0.0)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robotPose, false,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                            new TrcPose2D(robotPose.x, robotSafeY, robotPose.angle));
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    else
                    {
                        sm.setState(State.DONE);
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
