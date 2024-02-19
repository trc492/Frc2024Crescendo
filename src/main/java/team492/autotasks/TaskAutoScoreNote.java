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
import TrcFrcLib.frclib.FrcPhotonVision;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team492.FrcAuto;
import team492.Robot;
import team492.RobotParams;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoScoreNote extends TrcAutoTask<TaskAutoScoreNote.State>
{
    private static final String moduleName = TaskAutoScoreNote.class.getSimpleName();

    public enum ScoreTarget
    {
        Speaker,
        Arm,
        Trap
    }   //enum ScoreTarget

    public enum State
    {
        START,
        FIND_APRILTAG,
        DRIVE_TO_APRILTAG,
        SCORE_NOTE,
        DONE
    }   //enum State

    private static class TaskParams
    {
        Alliance alliance;
        ScoreTarget targetType;
        boolean inAuto;
        boolean useVision;
        boolean relocalize;
        boolean shootInPlace;

        TaskParams(
            Alliance alliance, ScoreTarget targetType, boolean inAuto, boolean useVision, boolean relocalize,
            boolean shootInPlace)
        {
            this.alliance = alliance;
            this.targetType = targetType;
            this.inAuto = inAuto;
            this.useVision = useVision;
            this.relocalize = relocalize;
            this.shootInPlace = shootInPlace;
        }   //TaskParams

    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event;

    private String currOwner = null;
    private int aprilTagId = 0;
    private TrcPose2D relAprilTagPose = null;
    private Double visionExpiredTime = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoScoreNote(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        this.event = new TrcEvent(moduleName + ".event");
    }   //TaskAutoScoreNote

    /**
     * This method starts the auto-assist operation.
     *
     * @param alliance specifies the alliance color, can be null if score in place.
     * @param targetType specifies the score target type.
     * @param inAuto specifies true if this is called by autonomous, false otherwise.
     * @param useVision specifies true to use vision to detect AprilTag, false otherwise.
     * @param relocalize specifies true to use vision to relocalize robot, false otherwise.
     * @param shootInPlace specifies true to shoot target in-place (not driving to AprilTag).
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScore(
        Alliance alliance, ScoreTarget targetType, boolean inAuto, boolean useVision, boolean relocalize,
        boolean shootInPlace, TrcEvent completionEvent)
    {
        tracer.traceInfo(
            moduleName,
            "alliance=" + alliance +
            ", targetType=" + targetType +
            ", inAuto=" + inAuto +
            ", useVision=" + useVision +
            ", relocalize=" + relocalize +
            ", shootInPlace=" + shootInPlace +
            ", event=" + completionEvent);
        startAutoTask(
            State.START,
            new TaskParams(alliance, targetType, inAuto, useVision, relocalize, shootInPlace), completionEvent);
    }   //autoAssistScore

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
                ", intake=" + ownershipMgr.getOwner(robot.intake) +
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
                ", intake=" + ownershipMgr.getOwner(robot.intake) +
                ", shooter=" + ownershipMgr.getOwner(robot.shooter) +
                ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            robot.shooter.releaseExclusiveAccess(currOwner);
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
        robot.intake.cancel(currOwner);
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
                // TODO: Determine what AprilTag ID to look for according to alliance and ScoreTarget type.
                relAprilTagPose = null;
                if (taskParams.useVision && robot.photonVisionFront != null)
                {
                    tracer.traceInfo(moduleName, "Using AprilTag Vision.");
                    sm.setState(State.FIND_APRILTAG);
                }
                else
                {
                    // Not using AprilTag vision, skip vision and just score at current location.
                    tracer.traceInfo(moduleName, "Not using AprilTag Vision.");
                    sm.setState(State.DRIVE_TO_APRILTAG);
                }
                break;

            case FIND_APRILTAG:
                // Use vision to determine the appropriate AprilTag location.
                FrcPhotonVision.DetectedObject object = robot.photonVisionFront.getBestDetectedObject();
                if (object != null)
                {
                    int aprilTagId = object.target.getFiducialId();
                    if (/*RED*/aprilTagId == 3 || aprilTagId == 4 || /*BLUE*/aprilTagId == 7 || aprilTagId == 8)
                    {
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
                if (taskParams.shootInPlace)
                {
                    sm.setState(State.SCORE_NOTE);
                }
                else
                {
                    // Navigate robot to Apriltag.
                    tracer.traceInfo(
                        moduleName, state + ": RobotFieldPose=" + robot.robotDrive.driveBase.getFieldPosition());
                    if (relAprilTagPose == null)
                    {
                        // Not using vision or vision did not see AprilTag, just go to it blindly using odometry and
                        // its known location. If we are not in Autonomous, the robot may not know its location. In
                        // this case, we just warn the driver and quit.
                        if (taskParams.inAuto)
                        {
                            // TODO: Need to take into consideration of starting position.
                            int targetAprilTagId =
                                FrcAuto.autoChoices.getAlliance() == DriverStation.Alliance.Red ? 4 : 7;
                            TrcPose2D absRobotPose = robot.robotDrive.driveBase.getFieldPosition();
                            TrcPose2D absAprilTagPose =
                                robot.photonVisionFront.getAprilTagPose(targetAprilTagId).toPose2D();
                            relAprilTagPose = absAprilTagPose.subtractRelativePose(absRobotPose);
                            tracer.traceInfo(
                                moduleName,
                                "Drive to AprilTag %d using odometry (robotPose=%s, aprilTagPose=%s, relPose=%s).",
                                targetAprilTagId, absRobotPose, absAprilTagPose, relAprilTagPose);
                        }
                        else if (robot.ledIndicator != null)
                        {
                            // Tell the drivers vision doesn't see anything so they can score manually.
                            robot.ledIndicator.setPhotonDetectedObject(null);
                        }
                    }

                    if (relAprilTagPose != null)
                    {
                        // Account for end-effector offset from the camera.
                        relAprilTagPose.y -= 5.85;
                        // Maintain heading to be squared to the Speaker.
                        relAprilTagPose.angle =
                            FrcAuto.autoChoices.getAlliance() == DriverStation.Alliance.Red ? 0.0 : 180.0;
                        // We are right in front of the Speaker, so we don't need full power to approach it.
                        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.35);
                        robot.robotDrive.purePursuitDrive.start(
                            currOwner, event, 3.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                            relAprilTagPose);
                        sm.waitForSingleEvent(event, State.SCORE_NOTE);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                }
                break;

            case SCORE_NOTE:
                // Determine shooter speed and tilt angle according to the score target type.
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoScoreNote
