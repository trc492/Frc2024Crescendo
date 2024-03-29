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

package team492.autotasks;

import TrcCommonLib.trclib.TrcAutoTask;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcTrigger.TriggerMode;
import TrcFrcLib.frclib.FrcPhotonVision;
import team492.Robot;
import team492.RobotParams;
import team492.vision.PhotonVision.PipelineType;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoPickupFromGround extends TrcAutoTask<TaskAutoPickupFromGround.State>
{
    private static final String moduleName = TaskAutoPickupFromGround.class.getSimpleName();

    public enum State
    {
        START,
        DETECT_NOTE, 
        DRIVE_TO_NOTE,
        CHECK_INTAKE_COMPLETION,
        DONE
    }   //enum State

    private static class TaskParams
    {
        boolean useVision;
        boolean inAuto;

        TaskParams(boolean useVision, boolean inAuto)
        {
            this.useVision = useVision;
            this.inAuto = inAuto;
        }
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent intakeEvent;
    private final TrcEvent driveEvent;
    private final TrcEvent gotNoteEvent;

    private TaskParams taskParams = null;
    private String currOwner = null;
    private String driveOwner = null;
    private Double visionExpiredTime = null;
    private TrcPose2D notePose = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPickupFromGround(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        this.intakeEvent = new TrcEvent(moduleName + ".intakeEvent");
        this.driveEvent = new TrcEvent(moduleName + ".driveEvent");
        this.gotNoteEvent = new TrcEvent(moduleName + ".gotNoteEvent");
    }   //TaskAutoPickupFromGround

    /**
     * This method starts the auto-assist operation.
     *
     * @param useVision specifies true to use Vision to detect the target, false otherwise.
     * @param inAuto specifies true if called by Autonomous, false otherwise.
     */
    public void autoAssistPickup(boolean useVision, boolean inAuto, TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "useVision=" + useVision + ", inAuto=" + inAuto + ", event=" + completionEvent);
        taskParams = new TaskParams(useVision, inAuto);
        startAutoTask(State.START, taskParams, completionEvent);
    }   //autoAssistPickup

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        tracer.traceInfo(moduleName, "Canceling auto-assist.");
        stopAutoTask(false);
    }   //autoAssistCancel

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called by the super class to acquire ownership of all subsystems involved in the auto-assist
     * operation. This is typically done before starting an auto-assist operation.
     *
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership()
    {
        boolean success = ownerName == null ||
                          robot.intake.acquireExclusiveAccess(ownerName) &&
                          robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName);

        if (success)
        {
            currOwner = ownerName;
            driveOwner = ownerName;
            tracer.traceInfo(moduleName, "Successfully acquired subsystem ownerships for " + ownerName + ".");
        }
        else
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceWarn(
                moduleName,
                "Failed to acquire subsystem ownership (currOwner=" + currOwner +
                ", intake=" + ownershipMgr.getOwner(robot.intake) +
                ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
            releaseSubsystemsOwnership();
        }

        return success;
    }   //acquireSubsystemsOwnership

    /**
     * This method is called by the super class to release ownership of all subsystems involved in the auto-assist
     * operation. This is typically done if the auto-assist operation is completed or canceled.
     */
    @Override
    protected void releaseSubsystemsOwnership()
    {
        if (currOwner != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                moduleName,
                "Releasing subsystem ownership (currOwner=" + currOwner +
                ", intake=" + ownershipMgr.getOwner(robot.intake) +
                ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
            robot.intake.releaseExclusiveAccess(currOwner);
            if (driveOwner != null)
            {
                robot.robotDrive.driveBase.releaseExclusiveAccess(driveOwner);
                driveOwner = null;
            }
            currOwner = null;
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called by the super class to stop all the subsystems.
     */
    @Override
    protected void stopSubsystems()
    {
        tracer.traceInfo(moduleName, "Stopping subsystems.");
        robot.intake.unregisterEntryTriggerNotifyEvent();
        if (!taskParams.inAuto)
        {
            robot.intake.cancel(currOwner);
        }
        robot.robotDrive.cancel(driveOwner);
        taskParams = null;
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false if running the fast loop on the main robot thread.
     */
    @Override
    protected void runTaskState(
        Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                notePose = null;
                // Auto pickup from ground. If not using vision, just spin up the intake and let the driver manually
                // drive to it.
                if (taskParams.useVision && robot.photonVisionBack != null)
                {
                    tracer.traceInfo(moduleName, "***** Using Note Vision.");
                    robot.photonVisionBack.setPipeline(PipelineType.NOTE);
                    visionExpiredTime = null;
                    sm.setState(State.DETECT_NOTE);
                }
                else
                {
                    tracer.traceInfo(moduleName, "***** Not using Note Vision.");
                    sm.setState(State.DRIVE_TO_NOTE);
                }
                break;

            case DETECT_NOTE:
                // Use vision to find Note.
                FrcPhotonVision.DetectedObject object = robot.photonVisionBack.getBestDetectedObject();
                if (object != null)
                {
                    notePose = object.getObjectPose();
                    notePose.x = -notePose.x;
                    notePose.y = -notePose.y;
                    tracer.traceInfo(
                        moduleName, "***** Vision found Note: notePose=" + notePose +
                        ", robotPose=" + robot.robotDrive.driveBase.getFieldPosition());
                    sm.setState(State.DRIVE_TO_NOTE);
                }
                else if (visionExpiredTime == null)
                {
                    // Can't find Note, set a timeout and try again.
                    visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out, moving on.
                    tracer.traceInfo(moduleName, "***** No Note Found.");
                    sm.setState(State.DRIVE_TO_NOTE);
                }
                break;

            case DRIVE_TO_NOTE:
                if ((notePose == null || Math.abs(notePose.y) > RobotParams.Intake.noteDistanceThreshold) &&
                    taskParams.inAuto)
                {
                    // We are in auto and vision did not see any Note, quit.
                    tracer.traceInfo(
                        moduleName,
                        "***** Either Vision doesn't see Note or Note is too far away: notePose=" + notePose);
                    if (robot.ledIndicator != null)
                    {
                        robot.ledIndicator.setPhotonDetectedObject(null, null);
                    }
                    sm.setState(State.DONE);
                }
                else
                {
                    // Even if Vision did not see the Note, turn on Intake regardless, so that driver can just
                    // manually move forward to pick up the Note.
                    robot.intake.autoIntakeForward(
                        currOwner, 0.0, RobotParams.Intake.intakePower, 0.0, 0.0, intakeEvent, 0.0);
                    sm.addEvent(intakeEvent);
                    // Register entry trigger to release drive ownership early.
                    robot.intake.registerEntryTriggerNotifyEvent(TriggerMode.OnActive, gotNoteEvent);
                    sm.addEvent(gotNoteEvent);
                    if (notePose != null)
                    {
                        tracer.traceInfo(moduleName, "***** Approach Note.");
                        // notePose is the intermediate pose. Back it off a bit so we can turn to it and do a straight
                        // run to it.
                        notePose.y += 6;
                        robot.robotDrive.purePursuitDrive.start(
                            currOwner, driveEvent, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                            notePose, new TrcPose2D(0.0, -26.0, 0.0));
                        sm.addEvent(driveEvent);
                    }
                    else
                    {
                        // Did not detect Note, release drive ownership to let driver to drive manually.
                        tracer.traceInfo(moduleName, "***** Did not see Note, release drive ownership.");
                        robot.robotDrive.driveBase.releaseExclusiveAccess(driveOwner);
                        driveOwner = null;
                        if (robot.ledIndicator != null)
                        {
                            robot.ledIndicator.setPhotonDetectedObject(null, null);
                        }
                    }
                    sm.waitForEvents(State.CHECK_INTAKE_COMPLETION, false);
                }
                break;

            case CHECK_INTAKE_COMPLETION:
                boolean gotNote = robot.intake.hasObject();
                tracer.traceInfo(
                    moduleName,
                    "**** Check Intake: intakeEvent=" + intakeEvent +
                    ", gotNoteEvent=" + gotNoteEvent +
                    ", driveEvent=" + driveEvent +
                    ", gotNote=" + gotNote);
                if (gotNote)
                {
                    // Got the Note. Release drive ownership early so drivers can drive away.
                    robot.robotDrive.purePursuitDrive.cancel(driveOwner);
                    robot.robotDrive.driveBase.releaseExclusiveAccess(driveOwner);
                    driveOwner = null;
                    // Entry trigger caused early drive ownership release doesn't mean we are done.
                    // Keep waiting for Intake to complete the operation.
                    sm.waitForSingleEvent(intakeEvent, State.DONE, 1.0);
                }
                else
                {
                    sm.setState(State.DONE);
                }
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoPickupFromGround
