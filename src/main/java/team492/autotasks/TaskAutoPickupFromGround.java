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
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcOwnershipMgr;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcPose3D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcPixyCam1.ObjectBlock;
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
        DONE
    }   //enum State

    private static class TaskParams
    {
        boolean inAuto;
        TaskParams(boolean inAuto)
        {
            this.inAuto = inAuto;
        }   //TaskParams
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent driveEvent;
    private final TrcEvent intakeEvent;
    


    private final TrcDbgTrace msgTracer;
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
    public TaskAutoPickupFromGround(String ownerName, Robot robot, TrcDbgTrace msgTracer)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        this.msgTracer = msgTracer;
        this.driveEvent = new TrcEvent(moduleName + ".driveEvent");
        this.intakeEvent = new TrcEvent(moduleName +".intakeEvent");
        // Intake event
        // Drive event
        
        // Vision Event




    }   //TaskAutoPickupFromGround

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoAssistPickup(TrcEvent completionEvent, boolean inAuto)
    {
        tracer.traceInfo(
            moduleName, 
            "event=" + completionEvent + 
            ", inAuto=" + inAuto);
        startAutoTask(State.START, new TaskParams(inAuto), completionEvent);
    }   //autoAssistPickup

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void autoAssistCancel()
    {
        final String funcName = "autoAssistCancel";

        if(msgTracer != null){
            tracer.traceInfo(moduleName, "Canceling auto-assist.");
        }
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
        // TODO: Figure out subsystem owenership aquisition
        boolean success = ownerName == null ||
                          (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName));

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
        // TODO: Figure out subsystem ownership release
        if (ownerName != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                moduleName,
                "Releasing subsystem ownership (currOwner=" + currOwner +
                ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }

        if (driveOwner != null)
        {
            robot.robotDrive.driveBase.releaseExclusiveAccess(driveOwner);
            driveOwner = null;
        }


        robot.intake.releaseExclusiveAccess(currOwner);
        currOwner = null;
    }   //releaseSubsystemsOwnership

    /**
     * This method is called by the super class to stop all the subsystems.
     */
    @Override
    protected void stopSubsystems()
    {
        tracer.traceInfo(moduleName, "Stopping subsystems.");
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
                // We are checking if the back camera is working, and if so, moving to the DETECT_NOTE state

                if(robot.photonVisionBack != null){
                    robot.photonVisionBack.setPipeline(PipelineType.NOTE);
                    tracer.traceInfo(moduleName, "Using Vision to find Note.");
                    sm.setState(State.DETECT_NOTE);
                } else{
                    // Back camera is not working, moving to DONE State
                    tracer.traceInfo(moduleName, "Error in Camera, moving to DONE State");
                    sm.setState(State.DONE);
                }

                break;

            case DETECT_NOTE:
                    
                FrcPhotonVision.DetectedObject object = robot.photonVisionBack.getBestDetectedObject();
                
                if(object != null){

                    // Note found with Vision, will move to PREP_TO_DRIVE STATE  
                    notePose = object.getObjectPose().toPose2D();

                    sm.setState(State.DRIVE_TO_NOTE);

                } else if (visionExpiredTime == null)
                {
                        // Can't find AprilTag, set a timeout and try again.
                        visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                } else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                        // Timed out, moving on.
                        tracer.traceInfo(moduleName, "No Note Found.");
                        sm.setState(State.DONE);
                }
                break;

            case DRIVE_TO_NOTE:

                
                // TODO: Write State
                // Will implement driving code

                if(notePose != null){

                    robot.intake.setPower(ownerName, RobotParams.Intake.intakePower);
                    robot.intake.registerEntryEvent(intakeEvent);
                    sm.addEvent(intakeEvent);
                    robot.robotDrive.purePursuitDrive.start(ownerName, driveEvent, 0.0, robot.robotDrive.driveBase.getFieldPosition(),true, notePose);
                    sm.addEvent(driveEvent);
                    sm.waitForEvents(State.DONE, false);

                } else{

                    sm.setState(State.DONE);

                }


                break;


            default:
            case DONE:
                // Stop task.

                robot.robotDrive.purePursuitDrive.cancel();

                robot.intake.setPower(ownerName, 0.0);
                sm.addEvent(null);

                if(robot.ledIndicator !=null){
                    
                    robot.ledIndicator.setIntakeDetectedObject(robot.intake.getNumObjects() > 0);

                }

            
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoPickupFromGround
