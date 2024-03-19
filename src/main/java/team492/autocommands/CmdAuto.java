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

import TrcCommonLib.trclib.TrcDbgTrace;
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
import team492.FrcAuto.ScoreWingNotes;
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
        SCORE_WING_NOTE_TO_AMP,
        CLEAR_THE_POST,
        TURN_TO_SPEAKER,
        TURN_TO_CENTERLINE,
        SCORE_NOTE_TO_SPEAKER,
        TURN_TO_WING_NOTES,
        DRIVE_TO_CENTER_LINE,
        PERFORM_END_ACTION,
        PICKUP_CENTERLINE_NOTE,
        DRIVE_TO_SPEAKER,
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
    private ScoreWingNotes scoreWingNotes;
    private EndAction endAction;
    private boolean relocalize;
    private int numWingNotesScored = 0;
    private int numCenterlineNotesScored = 0;
    private boolean aprilTagVisionEnabled = false;
    private boolean noteVisionEnabled = false;
    private boolean performingEndAction = false;

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
                if (noteObj.targetPose.y > RobotParams.Intake.noteDistanceThreshold ||
                    Math.abs(noteObj.targetPose.angle) > RobotParams.Intake.noteAngleThreshold)
                {
                    robot.globalTracer.traceDebug(
                        moduleName, "Vision found note too far or not turn enough at " + noteObj.targetPose + ".");
                    noteEvent.clear();
                }
                else
                {
                    robot.globalTracer.traceInfo(moduleName, "***** Vision found note at " + noteObj.targetPose + ".");
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
            TrcPose2D intermediatePose;

            robot.dashboard.displayPrintf(8, "State: " + state);
            robot.globalTracer.tracePreStateInfo(sm.toString(), state);
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
                    robot.shooter.tracer.setTraceLevel(TrcDbgTrace.MsgLevel.DEBUG);
                    // Shoot pre-load.
                    robot.globalTracer.traceInfo(
                        moduleName, "***** Shoot Preload from " + startPos +
                        " at " + robot.robotDrive.driveBase.getFieldPosition() + ".");
                    if (startPos == AutoStartPos.AMP)
                    {
                        robot.autoScoreNote.autoAssistScore(TargetType.Amp, true, true, relocalize, event);
                    }
                    else
                    {
                        robot.shooter.aimShooter(
                            null, RobotParams.Shooter.shooterSpeakerCloseVelocity,
                            RobotParams.Shooter.tiltSpeakerCloseAngle, 0.0, event, 0.0, robot::shoot, 0.0);
                    }
                    sm.waitForSingleEvent(event, State.DO_DELAY);
                    break;

                case DO_DELAY:
                    // Do delay if there is one.
                    robot.globalTracer.traceInfo(moduleName, "***** Set shooter to Wing Note preset.");
                    robot.shooter.aimShooter(
                        RobotParams.Shooter.wingNotePresetParams.shooterVelocity,
                        RobotParams.Shooter.wingNotePresetParams.tiltAngle, 0.0);

                    double startDelay = autoChoices.getStartDelay();
                    if (startDelay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + startDelay + "s.");
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_WING_NOTE);
                    }
                    else
                    {
                        sm.setState(State.DRIVE_TO_WING_NOTE);
                    }
                    break;

                case DRIVE_TO_WING_NOTE:
                    if (scoreWingNotes == ScoreWingNotes.SCORE_NONE)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** No Scoring Wing Note.");
                        sm.setState(State.DRIVE_TO_CENTER_LINE);
                    }
                    else
                    {
                        // For SW_AMP_SIDE or SW_SOURCE_SIDE, we need to get to the position where the camera can
                        // see the Note.
                        if(startPos == AutoStartPos.SW_SOURCE_SIDE || startPos == AutoStartPos.SW_AMP_SIDE)
                        {
                            robot.globalTracer.traceInfo(
                                moduleName,
                                "***** Drive to position so camera can see Wing Note, turn on Note Vision.");
                            robotPose = robot.robotDrive.driveBase.getFieldPosition();
                            TrcPose2D wingNotePose =
                                RobotParams.Game.wingNotePoses[0][startPos == AutoStartPos.SW_SOURCE_SIDE? 0: 2]
                                .clone();
                            wingNotePose.angle = 180.0;
                            intermediatePose = wingNotePose.clone();
                            intermediatePose.y -= 36.0;
                            robot.robotDrive.purePursuitDrive.start(
                                event, robotPose, false,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                                robot.adjustPoseByAlliance(intermediatePose, alliance),
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
                    robot.globalTracer.traceInfo(
                        moduleName, "***** Pickup Wing Note: Cancel PurePursuit, turn off Note Vision.");
                    robot.robotDrive.purePursuitDrive.cancel();
                    noteVisionEnabled = false;
                    robot.autoPickupFromGround.autoAssistPickup(true, true, event);
                    sm.waitForSingleEvent(
                        event,
                        startPos == AutoStartPos.AMP && numWingNotesScored == 0? State.SCORE_WING_NOTE_TO_AMP:
                        startPos == AutoStartPos.SW_SOURCE_SIDE && numWingNotesScored == 0?
                            State.CLEAR_THE_POST: State.TURN_TO_SPEAKER);
                    break;

                case SCORE_WING_NOTE_TO_AMP:
                    robot.globalTracer.traceInfo(moduleName, "***** Score first Wing Note to Amp.");
                    robot.autoScoreNote.autoAssistScore(TargetType.Amp, true, true, relocalize, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_WING_NOTES);
                    numWingNotesScored++;
                    break;

                case CLEAR_THE_POST:
                    // Scoring the first Wing Note from the Source side.
                    robot.globalTracer.traceInfo(
                        moduleName, "***** Too close to stage post, move a bit forward to clear it.");
                    robot.robotDrive.purePursuitDrive.start(
                        event, 0.5, robot.robotDrive.driveBase.getFieldPosition(), true,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                        new TrcPose2D(0.0, 24.0, 0.0));
                    sm.waitForSingleEvent(event, State.TURN_TO_SPEAKER);
                    break;

                case TURN_TO_SPEAKER:
                    if (!robot.intake.hasObject())
                    {
                        // Failed to pick up a Note, probably vision failed, go to EndAction.
                        robot.globalTracer.traceInfo(moduleName, "***** Failed to pick up a Note, go to EndAction.");
                        sm.setState(State.TURN_TO_CENTERLINE);
                    }
                    else if (numWingNotesScored > 0)
                    {
                        // We are scoring the 2nd or 3rd Wing Note to the Speaker, so must turn towards the Speaker.
                        robot.globalTracer.traceInfo(
                            moduleName, "***** Turn to Speaker to score: Wing Note " + (numWingNotesScored + 1) +
                            " or Centerline Note " + (numCenterlineNotesScored + 1) + ".");
                        robotPose = robot.robotDrive.driveBase.getFieldPosition();
                        targetPose = robotPose.clone();
                        targetPose.angle = alliance == Alliance.Red? 0.0: 180.0;
                        if (Math.abs(targetPose.x + RobotParams.Field.WIDTH) < RobotParams.Field.MIDFIELD_THRESHOLD)
                        {
                            // Scoring the third Wing Note from the Amp side, need to avoid the stage post.
                            intermediatePose = targetPose.clone();
                            intermediatePose.y += alliance == Alliance.Red? 12.0: -12.0;
                            robot.globalTracer.traceInfo(
                                moduleName, "***** Too close to the stage post at " + targetPose + ", move a bit forward.");
                            robot.robotDrive.purePursuitDrive.start(
                                event, robotPose, false,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                                intermediatePose, targetPose);
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(
                                moduleName, "***** Scoring 2nd or 3rd Wing Note at " + targetPose + ".");
                            robot.robotDrive.purePursuitDrive.start(
                                event, robotPose, false,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                                targetPose);
                        }
                        sm.addEvent(event);
                        aprilTagVisionEnabled = true;
                        sm.addEvent(aprilTagEvent);
                        sm.waitForEvents(State.SCORE_NOTE_TO_SPEAKER, false);
                    }
                    else
                    {
                        sm.setState(State.SCORE_NOTE_TO_SPEAKER);
                    }
                    break;

                case TURN_TO_CENTERLINE:
                    // We missed picking up a Note, go to EndAction but we need to turn to the Centerline first.
                    robot.globalTracer.traceInfo(
                        moduleName, "***** We missed picking up a Note, turn towards centerline to look for Note.");
                    robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    robot.robotDrive.purePursuitDrive.start(
                        event, robotPose, false,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                        new TrcPose2D(robotPose.x, robotPose.y, alliance == Alliance.Red? 0.0: 180.0));
                    sm.waitForSingleEvent(event, State.DRIVE_TO_CENTER_LINE);
                    break;

                case SCORE_NOTE_TO_SPEAKER:
                    robot.robotDrive.purePursuitDrive.cancel();
                    aprilTagVisionEnabled = false;
                    robot.autoScoreNote.autoAssistScore(TargetType.Speaker, true, true, relocalize, event);
                    if (performingEndAction)
                    {
                        numCenterlineNotesScored++;
                    }
                    else
                    {
                        numWingNotesScored++;
                    }
                    robot.globalTracer.traceInfo(
                        moduleName, "***** AutoScore Wing Note " + numWingNotesScored +
                        " or Centerline Note " + numCenterlineNotesScored + " to Speaker.");
                    sm.waitForSingleEvent(
                        event, performingEndAction? State.DRIVE_TO_CENTER_LINE: State.TURN_TO_WING_NOTES);
                    break;

                case TURN_TO_WING_NOTES:
                    // If we start at SW_CENTER, we don't do 2nd and 3rd Wing Note.
                    if (startPos != AutoStartPos.SW_CENTER &&
                        (scoreWingNotes == ScoreWingNotes.SCORE_TWO && numWingNotesScored < 2 ||
                         scoreWingNotes == ScoreWingNotes.SCORE_THREE && numWingNotesScored < 3))
                    {
                        if (startPos == AutoStartPos.AMP && numWingNotesScored == 1)
                        {
                            robot.globalTracer.traceInfo(
                                moduleName, "***** Finished scoring 1st Wing Note to Amp, already facing Wing Note.");
                            sm.setState(State.PICKUP_WING_NOTE);
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(
                                moduleName, "***** Turn to face Wing Note.");
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
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Done scoring Wing Notes.");
                        sm.setState(State.DRIVE_TO_CENTER_LINE);
                    }
                    break;

                case DRIVE_TO_CENTER_LINE:
                    if (endAction == EndAction.JUST_STOP ||
                        endAction == EndAction.SCORE_ONE_NOTE && numCenterlineNotesScored == 1 ||
                        endAction == EndAction.SCORE_TWO_NOTES && numCenterlineNotesScored == 2)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** EndAction is " + endAction + ", we are done.");
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(
                            moduleName, "***** EndAction is " + endAction + ", drive to Centerline.");
                        robotPose = robot.robotDrive.driveBase.getFieldPosition();
                        targetPose =
                            RobotParams.Game.centerlineNotePickupPoses[startPos == AutoStartPos.SW_SOURCE_SIDE? 0: 1];
                        robot.robotDrive.purePursuitDrive.start(
                            event, robotPose, false,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                            robot.adjustPoseByAlliance(targetPose, alliance));
                        sm.addEvent(event);
                        if (endAction != EndAction.PARK_NEAR_CENTER_LINE)
                        {
                            robot.globalTracer.traceInfo(moduleName, "***** Turn on Vision.");
                            noteVisionEnabled = true;
                            sm.addEvent(noteEvent);
                        }
                        sm.waitForEvents(State.PERFORM_END_ACTION, false);
                    }
                    break;

                case PERFORM_END_ACTION:
                    robot.globalTracer.traceInfo(moduleName, "***** Turn off Vision and cancel PurePursuitDrive.");
                    performingEndAction = true;
                    robot.robotDrive.purePursuitDrive.cancel();
                    noteVisionEnabled = false;
                    sm.setState(endAction == EndAction.PARK_NEAR_CENTER_LINE? State.DONE: State.PICKUP_CENTERLINE_NOTE);
                    break;

                case PICKUP_CENTERLINE_NOTE:
                    robot.globalTracer.traceInfo(moduleName, "***** Pickup Centerline Note.");
                    robot.autoPickupFromGround.autoAssistPickup(true, true, event);
                    sm.waitForSingleEvent(
                        event, endAction == EndAction.HOARD_ONE_NOTE? State.PARK: State.DRIVE_TO_SPEAKER);
                    break;

                case DRIVE_TO_SPEAKER:
                    robot.globalTracer.traceInfo(moduleName, "***** Drive to Speaker to score Centerline Note.");
                    robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    if (startPos == AutoStartPos.SW_SOURCE_SIDE)
                    {
                        targetPose = RobotParams.Game.centerlineNoteScorePoses[0];
                        intermediatePose = RobotParams.Game.centerlineNotePickupPoses[0];
                    }
                    else
                    {
                        targetPose = RobotParams.Game.centerlineNoteScorePoses[1];
                        intermediatePose = RobotParams.Game.centerlineNotePickupPoses[1];
                    }
                    robot.robotDrive.purePursuitDrive.start(
                        event, robotPose, false,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                        robot.adjustPoseByAlliance(intermediatePose, alliance),
                        robot.adjustPoseByAlliance(targetPose, alliance));
                    sm.waitForSingleEvent(event, State.SCORE_NOTE_TO_SPEAKER);
                    break;

                case PARK:
                    // Make sure we are back on our side if we went over just a little.
                    robot.globalTracer.traceInfo(
                        moduleName, "***** Back up a little to make sure we don't cross Centerline.");
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
            robot.globalTracer.tracePostStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAuto
