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
import edu.wpi.first.math.geometry.Pose3d;
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

    public enum TargetType
    {
        Speaker,
        Amp
    }   //enum TargetType

    public enum State
    {
        START,
        FIND_APRILTAG,
        DRIVE_TO_APRILTAG,
        AIM_IN_PLACE,
        SCORE_NOTE,
        DONE
    }   //enum State

    private static class TaskParams
    {
        TargetType targetType;
        boolean useVision;
        boolean inAuto;
        boolean relocalize;

        TaskParams(TargetType targetType, boolean useVision, boolean inAuto, boolean relocalize)
        {
            this.targetType = targetType;
            this.useVision = useVision;
            this.inAuto = inAuto;
            this.relocalize = relocalize;
        }   //TaskParams

    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event;

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
     * @param targetType specifies the score target type.
     * @param useVision specifies true to use Vision to detect the target, false otherwise.
     * @param inAuto specifies true if called by Autonomous, false otherwise.
     * @param relocalize specifies true to use AprilTag to relocalize, only valid if useVision is true.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistScore(
        TargetType targetType, boolean useVision, boolean inAuto, boolean relocalize,
        TrcEvent completionEvent)
    {
        tracer.traceInfo(
            moduleName,
            "targetType=" + targetType +
            ", useVision=" + useVision +
            ", inAuto=" + inAuto +
            ", relocalize=" + relocalize +
            ", event=" + completionEvent);
        startAutoTask(
            State.START, new TaskParams(targetType, useVision, inAuto, relocalize), completionEvent);
    }   //autoAssistScore

    /**
     * This method starts the auto-assist operation.
     *
     * @param targetType specifies the score target type.
     * @param useVision specifies true to use Vision to detect the target, false otherwise.
     */
    public void autoAssistScore(TargetType targetType, boolean useVision)
    {
        autoAssistScore(targetType, useVision, false, false, null);
    }   //autoAsistScore

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
            robot.intake.releaseExclusiveAccess(currOwner);
            robot.shooter.releaseExclusiveAccess(currOwner);
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
        robot.intake.cancel(currOwner);
        robot.shooter.cancel(currOwner);
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
                if (taskParams.useVision && robot.photonVisionFront != null)
                {
                    tracer.traceInfo(moduleName, "Using AprilTag Vision.");
                    // Get the Tilter out of the way of the camera.
                    robot.shooter.setTiltAngle(currOwner, RobotParams.Shooter.tiltTurtleAngle, event, 0.0);
                    sm.waitForSingleEvent(event, State.FIND_APRILTAG);
                }
                else
                {
                    // Not using AprilTag vision, skip vision and just score at current location.
                    tracer.traceInfo(moduleName, "Not using AprilTag Vision.");
                    sm.setState(State.SCORE_NOTE);
                }
                break;

            case FIND_APRILTAG:
                // Use vision to determine the appropriate AprilTag location.
                FrcPhotonVision.DetectedObject object =
                    robot.photonVisionFront.getBestDetectedAprilTag(
                        taskParams.targetType == TargetType.Speaker? new int[] {4, 7, 3, 8}: new int[] {5, 6});
                if (object != null)
                {
                    aprilTagId = object.target.getFiducialId();
                    tracer.traceInfo(
                        moduleName, "Vision found AprilTag %d at %s from camera.", aprilTagId, object.targetPose);
                    if (aprilTagId == 3 || aprilTagId == 8)
                    {
                        aprilTagPose = object.addTransformToTarget(
                            object.target, RobotParams.Vision.robotToFrontCam,
                            aprilTagId == 3? robot.aprilTag3To4Transform: robot.aprilTag8To7Transform);
                        tracer.traceInfo(
                            moduleName, "Translated AprilTag target at %s from camera.", aprilTagPose);
                    }
                    else
                    {
                        aprilTagPose = object.getObjectPose();
                    }
                    // Relocalize.
                    if (taskParams.relocalize)
                    {
                        TrcPose2D robotFieldPose =
                            robot.photonVisionFront.getRobotFieldPose(object, true);
                        // If we see the AprilTag, we can use its location to re-localize the robot.
                        if (robotFieldPose != null)
                        {
                            robot.robotDrive.driveBase.setFieldPosition(robotFieldPose, false);
                            tracer.traceInfo(moduleName, "Using AprilTag to re-localize to " + robotFieldPose);
                        }
                    }
                    sm.setState(State.DRIVE_TO_APRILTAG);
                }
                else if (visionExpiredTime == null)
                {
                    // Can't find AprilTag, set a timeout and try again.
                    // TODO: re-evaluate timeout!
                    visionExpiredTime = TrcTimer.getCurrentTime() + 2.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out, moving on.
                    tracer.traceInfo(moduleName, "No AprilTag found.");
                    sm.setState(State.DRIVE_TO_APRILTAG);
                }
                break;

            case DRIVE_TO_APRILTAG:
                if (taskParams.targetType == TargetType.Speaker)
                {
                    sm.setState(State.AIM_IN_PLACE);
                }
                else if (aprilTagPose != null)
                {
                    TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    Pose3d aprilTagFieldPose3d = robot.photonVisionFront.getAprilTagFieldPose3d(aprilTagId);
                    TrcPose2D targetPose = robot.photonVisionFront.getTargetPoseOffsetFromAprilTag(
                        aprilTagFieldPose3d, 0.0, -RobotParams.Robot.LENGTH / 2.0, -90.0);
                    tracer.traceInfo(
                        moduleName,
                        state + ": RobotFieldPose=" + robotPose +
                        "\n\taprilTagPose=" + aprilTagPose +
                        "\n\ttargetPose=" + targetPose);
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robotPose, false,
                        RobotParams.SwerveDriveBase.ROBOT_MAX_VELOCITY,
                        RobotParams.SwerveDriveBase.ROBOT_MAX_ACCELERATION,
                        targetPose);
                    sm.waitForSingleEvent(event, State.SCORE_NOTE);
                }
                else
                {
                    // Did not detect AprilTag, tell the drivers so they can score manually if in TeleOp and quit.
                    // If in Auto, shoot it to get rid of the Note anyway or we won't be able to get the Wing Notes.
                    // We could also shoot it to Speaker, but it complicates the logic a lot.
                    if (robot.ledIndicator != null)
                    {   
                        robot.ledIndicator.setPhotonDetectedObject(null);
                    }
                    sm.setState(taskParams.inAuto? State.SCORE_NOTE: State.DONE);
                }
                break;

            case AIM_IN_PLACE:
                if (aprilTagPose != null || taskParams.inAuto)
                {
                    // If we are up against the subwoofer, PurePursuitDrive will get stuck. Set a timeout to get out
                    // of this situation.
                    // If we are in auto and did not see AprilTag, just align to the field and shoot blind.
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        RobotParams.SwerveDriveBase.ROBOT_MAX_VELOCITY,
                        RobotParams.SwerveDriveBase.ROBOT_MAX_ACCELERATION,
                        new TrcPose2D(0.0, 0.0,
                            aprilTagPose != null? aprilTagPose.angle:
                            FrcAuto.autoChoices.getAlliance() == Alliance.Red? 0.0: 180.0));
                    sm.waitForSingleEvent(event, State.SCORE_NOTE);
                }
                else
                {
                    // Did not detect AprilTag, tell the drivers so they can score manually if in TeleOp and quit.
                    if (robot.ledIndicator != null)
                    {
                        robot.ledIndicator.setPhotonDetectedObject(null);
                    }
                    sm.setState(State.DONE);
                }
                break;

            case SCORE_NOTE:
                // Determine shooter speed and tilt angle according to the score target type.
                double shooterVel = 0.0;
                double tiltAngle = 0.0;

                switch (taskParams.targetType)
                {
                    case Speaker:
                        if (aprilTagPose != null)
                        {
                            // Use vision distance to look up shooter parameters.
                            ShootParamTable.Params shootParams =
                                RobotParams.Shooter.speakerShootParamTable.get(aprilTagPose.y);
                            tracer.traceInfo(
                                moduleName, "ShootParams: distance=" + aprilTagPose.y + ", params=" + shootParams);
                            shooterVel = shootParams.shooterVelocity;
                            tiltAngle = shootParams.tiltAngle;
                        }
                        else if (!taskParams.useVision)
                        {
                            // Did not use vision, must be shooting at Speaker close-up.
                            shooterVel = RobotParams.Shooter.shooterSpeakerCloseVelocity;
                            tiltAngle = RobotParams.Shooter.tiltSpeakerCloseAngle;
                        }
                        break;

                    case Amp:
                        shooterVel = RobotParams.Shooter.shooterAmpVelocity;
                        tiltAngle = RobotParams.Shooter.tiltAmpAngle;
                        break;
                }
                robot.shooter.aimShooter(currOwner, shooterVel, tiltAngle, 0.0, event, 0.0, robot::shoot, 1.0);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoScoreNote
