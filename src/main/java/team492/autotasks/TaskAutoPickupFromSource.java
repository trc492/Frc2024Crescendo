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
import edu.wpi.first.math.geometry.Pose3d;
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
        CHECK_INTAKE_COMPLETION,
        DONE
    }   //enum State

    private static class TaskParams
    {
        boolean useVision;

        TaskParams(boolean useVision)
        {
            this.useVision = useVision;
        }
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent intakeEvent;
    private final TrcEvent driveEvent;
    private final TrcEvent gotNoteEvent;

    private String currOwner = null;
    private String driveOwner = null;
    private Double visionExpiredTime = null;
    private int aprilTagId = -1;
    private TrcPose2D aprilTagPose = null;

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
        this.intakeEvent = new TrcEvent(moduleName + "intakeEvent");
        this.driveEvent = new TrcEvent(moduleName + ".driveEvent");
        this.gotNoteEvent = new TrcEvent(moduleName + ".gotNoteEvent");
    }   //TaskAutoPickupFromSource

    /**
     * This method starts the auto-assist operation.
     *
     * @param useVision specifies true to use Vision to detect the target, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistPickup(boolean useVision, TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "useVision=" + useVision + ", event=" + completionEvent);
        startAutoTask(State.START, new TaskParams(useVision), completionEvent);
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
                ", shooter=" + ownershipMgr.getOwner(robot.shooter) +
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
                ", shooter=" + ownershipMgr.getOwner(robot.shooter) +
                ", intake=" + ownershipMgr.getOwner(robot.intake) +
                ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
            robot.shooter.releaseExclusiveAccess(currOwner);
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
        robot.intake.unregisterExitTriggerNotifyEvent();
        robot.shooter.cancel(currOwner);
        robot.intake.cancel(currOwner);
        robot.robotDrive.cancel(driveOwner);
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
                aprilTagId = -1;
                aprilTagPose = null;
                // Shooter takes time to spin up and aim, so start it the first thing.
                robot.shooter.aimShooter(
                    currOwner, RobotParams.Shooter.shooterSourcePickupVelocity,
                    RobotParams.Shooter.tiltSourcePickupAngle, 0.0, null, 0.0, null);
                robot.intake.autoIntakeReverse(
                    currOwner, 0.0, RobotParams.Intake.intakePower, 0.0, 0.0, intakeEvent, 0.0);
                // Auto pickup from source must use vision. If vision is not available, quit.
                if (taskParams.useVision && robot.photonVisionFront != null)
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
                // There are two AprilTags at the source, we will specify preferences of which one to pick.
                FrcPhotonVision.DetectedObject object = robot.photonVisionFront.getBestDetectedAprilTag(1, 10, 2, 9);
                if (object != null)
                {
                    aprilTagId = object.target.getFiducialId();
                    aprilTagPose = object.getObjectPose();
                    tracer.traceInfo(
                        moduleName, "Vision found AprilTag %d at %s from camera.", aprilTagId, aprilTagPose);
                    // Relocalize.
                    TrcPose2D robotFieldPose =
                        robot.photonVisionFront.getRobotFieldPose(object, true);
                    // If we see the AprilTag, we can use its location to re-localize the robot.
                    if (robotFieldPose != null)
                    {
                        robot.robotDrive.driveBase.setFieldPosition(robotFieldPose, false);
                        tracer.traceInfo(moduleName, "Using AprilTag to re-localize to " + robotFieldPose);
                    }
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
                sm.addEvent(intakeEvent);
                robot.intake.registerExitTriggerNotifyEvent(TriggerMode.OnActive, gotNoteEvent);
                sm.addEvent(gotNoteEvent);
                if (aprilTagPose != null)
                {
                    TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    double targetAngle = aprilTagId > 2? 120.0: 30.0;
                    Pose3d aprilTagFieldPose3d = robot.photonVisionFront.getAprilTagFieldPose3d(aprilTagId);
                    TrcPose2D targetPose = robot.photonVisionFront.getTargetPoseOffsetFromAprilTag(
                        aprilTagFieldPose3d, 0.0, -RobotParams.Robot.LENGTH / 2.0, targetAngle);
                    tracer.traceInfo(
                        moduleName,
                        state + ": RobotFieldPose=" + robotPose +
                        "\n\taprilTagPose=" + aprilTagPose +
                        "\n\ttargetPose=" + targetPose);
                    // We are right in front of the target, so we don't need full power to approach it.
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.3);
                    robot.robotDrive.purePursuitDrive.start(currOwner, driveEvent, 3.0, robotPose, false, targetPose);
                    sm.addEvent(driveEvent);
                }
                else
                {
                    // Did not detect AprilTag, release drive ownership to let driver to drive manually.
                    robot.robotDrive.driveBase.releaseExclusiveAccess(driveOwner);
                    driveOwner = null;
                    if (robot.ledIndicator != null)
                    {
                        robot.ledIndicator.setPhotonDetectedObject(null);
                    }
                }
                sm.waitForEvents(State.CHECK_INTAKE_COMPLETION, true);
                break;

            case CHECK_INTAKE_COMPLETION:
                boolean gotNote = robot.intake.hasObject();
                if (robot.ledIndicator !=null)
                {
                    robot.ledIndicator.setIntakeDetectedObject(gotNote);
                }

                if (gotNote)
                {
                    // Got the Note. Release drive ownership early so drivers can drive away.
                    robot.robotDrive.driveBase.releaseExclusiveAccess(driveOwner);
                    driveOwner = null;
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

}   //class TaskAutoPickupFromSource
