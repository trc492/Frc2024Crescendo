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
    private final TrcEvent autoAssistDriveEvent;
    private final TrcEvent autoAssistScoreEvent;
    private final TrcStateMachine<State> sm;

    private Alliance alliance;
    private int startPos;
    private boolean scoreWingNote;
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
        autoAssistDriveEvent = new TrcEvent(moduleName + "autoAssistEvent");
        autoAssistScoreEvent = new TrcEvent(moduleName + "autoAssistEvent");
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

                    alliance = FrcAuto.autoChoices.getAlliance();
                    startPos = FrcAuto.autoChoices.getStartPos();
                    scoreWingNote = FrcAuto.autoChoices.getScoreWingNote();
                    TrcPose2D startPose = RobotParams.startPos[alliance == Alliance.Red ? 0: 1][startPos];
                    robot.robotDrive.driveBase.setFieldPosition(startPose);



                
                    

                    TargetType targetType = TargetType.Speaker;
                    boolean shootInPlace = true;

                    if(startPos == 0){
                        targetType = TargetType.Amp;
                        shootInPlace = false;


                    } 
                    
          

                    // Amp Starting Position
                    robot.autoScoreNote.autoAssistScore(
                        targetType,
                        true, 
                        shootInPlace, 
                        autoAssistScoreEvent);
                    // Drive to Wing Note, or, if we do not want to move to the Wing Note move to Park state
                    sm.waitForSingleEvent(autoAssistDriveEvent,  State.DO_DELAY);




                case DO_DELAY:
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


                    if(scoreWingNote){

                        if(startPos == 1 || startPos == 3){

                            TrcPose2D wingNotePose = RobotParams.wingNotePoses[1][startPos == 1 ? 0: 2].clone();
                            // Using Blue Alliance Side

                            wingNotePose.y -= 24; // TODO: Find Actual Distance
                            wingNotePose.angle = 180;
                            
                            robot.robotDrive.purePursuitDrive.start(autoAssistDriveEvent, robot.robotDrive.driveBase.getFieldPosition(), false, wingNotePose);
                            sm.waitForSingleEvent(autoAssistDriveEvent, State.PICKUP_WING_NOTE);

                        } else{
                            sm.setState(State.PICKUP_WING_NOTE);
                        }



                    } else{
                        sm.setState(State.PERFORM_END_ACTION);
                    }

                case PICKUP_WING_NOTE:

                    robot.autoPickupFromGround.autoAssistPickup(true, event); // TODO: Formalize event names, this event is for picking up a wing note
                    sm.waitForSingleEvent(event, State.SCORE_WING_NOTE);

                    break;

                case SCORE_WING_NOTE:


                    robot.autoScoreNote.autoAssistScore(TargetType.Speaker, true, true, event);
                    sm.waitForSingleEvent(event, State.PERFORM_END_ACTION);
                    break;


                case PERFORM_END_ACTION:
                
        

                    
                    TrcPose2D centerlineNotePose = RobotParams.centerlineNotePoses[0].clone();
                    
                    // Using Blue Alliance Side

                    centerlineNotePose.y -= 24; // TODO: Find Actual Distance
                    centerlineNotePose.angle = 180;
                    if(startPos == 3){
                        robot.robotDrive.purePursuitDrive.start(autoAssistDriveEvent, robot.robotDrive.driveBase.getFieldPosition(), false, 
                        RobotParams.WINGNOTE_BLUE_SW_SIDE, 
                        centerlineNotePose);
                        
                    } else{
                        robot.robotDrive.purePursuitDrive.start(autoAssistDriveEvent, robot.robotDrive.driveBase.getFieldPosition(), false, centerlineNotePose);
                    }

                    sm.waitForSingleEvent(autoAssistDriveEvent, State.PICKUP_CENTERLINE_NOTE);

                    


                    

                    break;
                case PICKUP_CENTERLINE_NOTE:
                
                    FrcAuto.EndAction endAction = FrcAuto.autoChoices.getEndAction();

                    if(endAction == FrcAuto.EndAction.PARK_NEAR_CENTERLINE){
                        sm.setState(State.DONE);
                    } else{
                        robot.autoPickupFromGround.autoAssistPickup(true, autoAssistDriveEvent);
                        sm.waitForSingleEvent(autoAssistDriveEvent, State.DONE);
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
