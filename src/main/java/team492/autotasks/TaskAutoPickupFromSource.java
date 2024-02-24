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
import TrcCommonLib.trclib.TrcIntake.TriggerType;
import TrcFrcLib.frclib.FrcPhotonVision;
import team492.Robot;
import team492.RobotParams;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoPickupFromSource extends TrcAutoTask<TaskAutoPickupFromSource.State>
{
    private static final String moduleName = TaskAutoPickupFromSource.class.getSimpleName();

    public enum State
    {
        START,
        FIND_APRILTAG,
        DRIVE_TO_APRILTAG,
        DONE
    }   //enum State

    private static class TaskParams
    {
        boolean relocalize;

        TaskParams(boolean relocalize)
        {
            this.relocalize = relocalize;
        }
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event;

    private String currOwner = null;
    private int aprilTagId = -1;
    private TrcPose2D relAprilTagPose = null;
    private Double visionExpiredTime = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPickupFromSource(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        this.event = new TrcEvent(moduleName + ".event");
    }   //TaskAutoPickupFromSource

    /**
     * This method starts the auto-assist operation.
     *
     * @param relocalize specifies true to use vision to relocalize robot, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistPickup(boolean relocalize, TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "relocalize=" + relocalize + ", event=" + completionEvent);
        startAutoTask(State.START, new TaskParams(relocalize), completionEvent);
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
                          robot.shooter.acquireExclusiveAccess(ownerName) &&
                          robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName);

        if (success)
        {
            currOwner = ownerName;
            tracer.traceInfo(moduleName, "Successfully acquired subsystem ownerships.");
        }
        else
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceWarn(
                moduleName,
                "Failed to acquire subsystem ownership (currOwner=" + currOwner +
                ", shooter=" + ownershipMgr.getOwner(robot.shooter) +
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
                ", shooter=" + ownershipMgr.getOwner(robot.shooter) +
                ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
            robot.shooter.releaseExclusiveAccess(currOwner);
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
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
        robot.intake.unregisterExitTriggerNotifyEvent();
        robot.shooter.cancel(currOwner);
        robot.robotDrive.cancel(currOwner);
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
                relAprilTagPose = null;
                // Shooter takes time to spin up and aim, so start it the first thing.
                robot.shooter.aimShooter(
                    currOwner, RobotParams.Shooter.shooterPickupVelocity, RobotParams.Shooter.tiltPickupAngle,
                    0.0, null, 0.0, null);
                // Auto pickup from source must use vision. If vision is not available, quit.
                if (robot.photonVisionFront != null)
                {
                    tracer.traceInfo(moduleName, "Using AprilTag Vision.");
                    sm.setState(State.FIND_APRILTAG);
                }
                else
                {
                    sm.setState(State.DRIVE_TO_APRILTAG);
                }
                break;

            case FIND_APRILTAG:
                // Use vision to look for any AprilTag and determine its location.
                // Even though there are two AprilTags at the source, we will let Vision pick the best one.
                FrcPhotonVision.DetectedObject object = robot.photonVisionFront.getDetectedAprilTag(-1);
                if (object != null)
                {
                    // We were looking for any AprilTag, get the detected AprilTag ID.
                    aprilTagId = object.target.getFiducialId();
                    if (taskParams.relocalize)
                    {
                        TrcPose2D robotFieldPose =
                            robot.photonVisionFront.getRobotFieldPosition(RobotParams.Vision.CAMERA_TRANSFORM3D);
                        // If we see the AprilTag, we can use its location to re-localize the robot.
                        robot.robotDrive.driveBase.setFieldPosition(robotFieldPose, false);
                        tracer.traceInfo(moduleName, "Using AprilTag to re-localize to " + robotFieldPose);
                    }
                    relAprilTagPose = object.targetPose.toPose2D();
                    tracer.traceInfo(
                        moduleName, "Vision found AprilTag %d at %s from camera.", aprilTagId, object.targetPose);
                    sm.setState(State.DRIVE_TO_APRILTAG);
                }
                else if (visionExpiredTime == null)
                {
                    // Can't find AprilTag, set a timeout and try again.
                    visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out, moving on.
                    tracer.traceInfo(moduleName, "No AprilTag found.");
                    sm.setState(State.DRIVE_TO_APRILTAG);
                }
                break;

            case DRIVE_TO_APRILTAG:
                if (relAprilTagPose != null)
                {
                    // Account for end-effector offset from the camera and heading.
                    relAprilTagPose.y -= 5.85; //TODO: Tune
                    // We are right in front of the Speaker, so we don't need full power to approach it.
                    tracer.traceInfo(
                        moduleName,
                        state + ": RobotFieldPose=" + robot.robotDrive.driveBase.getFieldPosition() +
                        ", RelAprilTagPose=" + relAprilTagPose);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.35);
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, null, 3.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        relAprilTagPose);
                }
                else if (robot.ledIndicator != null)
                {
                    // Did not detect AprilTag, tell the drivers so they can drive to the source manually.
                    robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
                    robot.ledIndicator.setPhotonDetectedObject(null);
                }
                robot.intake.registerExitTriggerNotifyEvent(event, TriggerType.TRIGGER_ACTIVE);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoPickupFromSource
