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

package team492;

import java.util.Arrays;

import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcDriveBase.DriveOrientation;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcJoystick;
import TrcFrcLib.frclib.FrcXboxController;
import team492.autotasks.TaskAutoScoreNote.TargetType;

/**
 * This class implements the code to run in TeleOp Mode.
 */
public class FrcTeleOp implements TrcRobot.RobotMode
{
    private static final String moduleName = FrcTeleOp.class.getSimpleName();
    private static final boolean traceButtonEvents = true;
    //
    // Global objects.
    //
    protected final Robot robot;
    private boolean controlsEnabled = false;
    protected boolean altFunc = false;
    // DriveBase subsystem.
    private double driveSpeedScale = RobotParams.DRIVE_NORMAL_SCALE;
    private double turnSpeedScale = RobotParams.TURN_NORMAL_SCALE;
    private double[] prevDriveInputs = null;
    // Shooter subsystem.
    private double prevShooterVel = 0.0;
    private double prevTiltPower = 0.0;
    // Climber subsystem.
    private double prevClimbPower = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object to access all robot hardware and subsystems.
     */
    public FrcTeleOp(Robot robot)
    {
        //
        // Create and initialize global object.
        //
        this.robot = robot;
    }   //FrcTeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    /**
     * This method is called when the teleop mode is about to start. Typically, you put code that will prepare
     * the robot for start of teleop here such as creating and configuring joysticks and other subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Enabling joysticks.
        //
        setControlsEnabled(true);
        //
        // Initialize subsystems for TeleOp mode if necessary.
        //
        if (robot.robotDrive != null)
        {
            robot.robotDrive.driveBase.setDriveOrientation(DriveOrientation.FIELD, true);
        }

        if (RobotParams.Preferences.hybridMode)
        {
            // This makes sure that the autonomous stops running when
            // teleop starts running. If you want the autonomous to
            // continue until interrupted by another command, remove
            // this line or comment it out.
            if (robot.m_autonomousCommand != null)
            {
                robot.m_autonomousCommand.cancel();
            }
        }
    }   //startMode

    /**
     * This method is called when teleop mode is about to end. Typically, you put code that will do clean
     * up here such as disabling joysticks and other subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Disabling joysticks.
        //
        setControlsEnabled(false);
        //
        // Disable subsystems before exiting if necessary.
        //
        robot.autoAssistCancel();
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        int lineNum = 1;

        if (slowPeriodicLoop)
        {
            if (controlsEnabled)
            {
                //
                // DriveBase operation.
                //
                if (robot.robotDrive != null)
                {
                    double[] driveInputs = robot.robotDrive.getDriveInputs(
                        RobotParams.ROBOT_DRIVE_MODE, true, driveSpeedScale, turnSpeedScale);
                    if (!Arrays.equals(driveInputs, prevDriveInputs))
                    {
                        if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                        {
                            robot.robotDrive.driveBase.holonomicDrive(
                                null, driveInputs[0], driveInputs[1], driveInputs[2],
                                robot.robotDrive.driveBase.getDriveGyroAngle());
                            robot.dashboard.displayPrintf(
                                lineNum++, "Holonomic: x=%.3f, y=%.3f, rot=%.3f",
                                driveInputs[0], driveInputs[1], driveInputs[2]);
                        }
                        else if (RobotParams.Preferences.useTankDrive)
                        {
                            robot.robotDrive.driveBase.tankDrive(driveInputs[0], driveInputs[1]);
                            robot.dashboard.displayPrintf(
                                lineNum++, "Tank: left=%.3f, right=%.3f, rot=%.3f",
                                driveInputs[0], driveInputs[1], driveInputs[2]);
                        }
                        else
                        {
                            robot.robotDrive.driveBase.arcadeDrive(driveInputs[1], driveInputs[2]);
                            robot.dashboard.displayPrintf(
                                lineNum++, "Arcade: x=%.3f, y=%.3f, rot=%.3f",
                                driveInputs[0], driveInputs[1], driveInputs[2]);
                        }
                    }
                    else
                    {
                        lineNum++;
                    }
                    prevDriveInputs = driveInputs;
                    robot.dashboard.displayPrintf(
                        lineNum++, "RobotPose=%s, Orient=%s, GyroAssist=%s",
                        robot.robotDrive.driveBase.getFieldPosition(),
                        robot.robotDrive.driveBase.getDriveOrientation(),
                        robot.robotDrive.driveBase.isGyroAssistEnabled());;
                }
                //
                // Analog control of subsystem is done here if necessary.
                //
                if (RobotParams.Preferences.useSubsystems)
                {
                    if (robot.intake != null)
                    {
                        robot.dashboard.displayPrintf(
                            lineNum++, "Intake: power=%.2f, entry/exit=%s/%s",
                            robot.intake.getPower(),
                            robot.intake.isTriggerActive(robot.intake.entryTrigger),
                            robot.intake.isTriggerActive(robot.intake.exitTrigger));
                    }

                    if (robot.shooter != null)
                    {
                        double shooterVel =
                            (robot.operatorController.getRightTriggerAxis() -
                             robot.operatorController.getLeftTriggerAxis()) * RobotParams.Shooter.shooterMaxVelocity;
                        // Only set shooter velocity if it is different from previous value.
                        if (prevShooterVel != shooterVel)
                        {
                            if (shooterVel == 0.0)
                            {
                                // Don't abruptly stop the shooter, gently spin down.
                                robot.shooter.stopShooter();
                            }
                            else
                            {
                                robot.shooter.setShooterVelocity(shooterVel);
                            }
                            prevShooterVel = shooterVel;
                        }
                        robot.dashboard.displayPrintf(
                            lineNum++, "Shooter: vel=%.0f/%.0f, preset=%.0f, inc=%.0f",
                            shooterVel, robot.shooter.getShooterVelocity(), robot.shooterVelocity.getValue(),
                            robot.shooterVelocity.getIncrement());

                        double tiltPower = robot.operatorController.getLeftYWithDeadband(true);
                        // Only set tilt power if it is different from previous value.
                        if (prevTiltPower != tiltPower)
                        {
                            robot.shooter.setTiltPower(tiltPower);
                            prevTiltPower = tiltPower;
                        }
                        robot.dashboard.displayPrintf(
                            lineNum++, "Tilt: power=%.2f/%.2f, angle=%.2f/%.2f/%f, inc=%.0f, limits=%s/%s",
                            tiltPower, robot.shooter.getTiltPower(), robot.shooter.getTiltAngle(),
                            robot.shooter.tiltMotor.getPidTarget(), robot.shooter.tiltMotor.getMotorPosition(),
                            robot.shooterTiltAngle.getIncrement(), robot.shooter.tiltLowerLimitSwitchActive(),
                            robot.shooter.tiltUpperLimitSwitchActive());
                    }

                    if (robot.climber != null)
                    {
                        double climbPower = robot.operatorController.getRightYWithDeadband(true);
                        if (prevClimbPower != climbPower)
                        {
                            robot.climber.setClimbPower(climbPower);
                            prevClimbPower = climbPower;
                        }
                        robot.dashboard.displayPrintf(
                            lineNum++, "Climber: power=%.2f/%.2f, current=%.3f, pos=%.2f/%.2f/%f, limits=%s/%s",
                            climbPower, robot.climber.climberMotor.getPower(), robot.climber.climberMotor.getCurrent(),
                            robot.climber.getPosition(), robot.climber.climberMotor.getPidTarget(),
                            robot.climber.climberMotor.getMotorPosition(),
                            robot.climber.climberMotor.isLowerLimitSwitchActive(),
                            robot.climber.climberMotor.isUpperLimitSwitchActive());
                    }
                }
            }
            //
            // Update robot status.
            //
            if (RobotParams.Preferences.doStatusUpdate)
            {
                robot.updateStatus();
            }
        }
    }   //periodic

    /**
     * This method enables/disables joystick controls.
     *
     * @param enabled specifies true to enable joystick control, false to disable.
     */
    protected void setControlsEnabled(boolean enabled)
    {
        controlsEnabled = enabled;

        if (RobotParams.Preferences.useDriverXboxController)
        {
            robot.driverController.setButtonHandler(enabled? this::driverControllerButtonEvent: null);
        }
        else
        {
            robot.leftDriveStick.setButtonHandler(enabled? this::leftDriveStickButtonEvent: null);
            robot.rightDriveStick.setButtonHandler(enabled? this::rightDriveStickButtonEvent: null);
        }

        if (RobotParams.Preferences.useOperatorXboxController)
        {
            robot.operatorController.setButtonHandler(enabled? this::operatorControllerButtonEvent: null);
        }
        else
        {
            robot.operatorStick.setButtonHandler(enabled? this::operatorStickButtonEvent: null);
        }

        if (RobotParams.Preferences.useButtonPanels)
        {
            robot.buttonPanel.setButtonHandler(enabled? this::buttonPanelButtonEvent: null);
            robot.switchPanel.setButtonHandler(enabled? this::switchPanelButtonEvent: null);
        }
    }   //setControlsEnabled

    //
    // Implements FrcButtonHandler.
    //

    /**
     * This method is called when a driver controller button event is detected.
     *
     * @param button specifies the button ID that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void driverControllerButtonEvent(int button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "DriverController: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcXboxController.BUTTON_A:
                // Toggle between field or robot oriented driving.
                if (robot.robotDrive != null && pressed)
                {
                    if (robot.robotDrive.driveBase.getDriveOrientation() != DriveOrientation.FIELD)
                    {
                        robot.robotDrive.driveBase.setDriveOrientation(DriveOrientation.FIELD, true);
                    }
                    else
                    {
                        robot.robotDrive.driveBase.setDriveOrientation(DriveOrientation.ROBOT, false);
                    }
                }
                break;

            case FrcXboxController.BUTTON_B:
                if (robot.robotDrive != null && pressed)
                {
                    boolean gyroAssistEnabled = !robot.robotDrive.driveBase.isGyroAssistEnabled();
                    robot.robotDrive.driveBase.setGyroAssistEnabled(
                        gyroAssistEnabled? robot.robotDrive.pidDrive.getTurnPidCtrl(): null);
                }
                break;

            case FrcXboxController.LEFT_BUMPER:
                if (pressed)
                {
                    driveSpeedScale = RobotParams.DRIVE_SLOW_SCALE;
                    turnSpeedScale = RobotParams.TURN_SLOW_SCALE;
                }
                else
                {
                    driveSpeedScale = RobotParams.DRIVE_NORMAL_SCALE;
                    turnSpeedScale = RobotParams.TURN_NORMAL_SCALE;
                }
                break;

            case FrcXboxController.RIGHT_BUMPER:
                if (robot.robotDrive != null &&
                    robot.robotDrive.driveBase.getDriveOrientation() == DriveOrientation.ROBOT)
                {
                    // Inverted drive only makes sense for robot oriented driving.
                    robot.robotDrive.driveBase.setDriveOrientation(
                        pressed? DriveOrientation.INVERTED: DriveOrientation.ROBOT, false);
                }
                break;

            case FrcXboxController.BACK:
                break;

            case FrcXboxController.START:
                break;

            case FrcXboxController.LEFT_STICK_BUTTON:
                break;

            case FrcXboxController.RIGHT_STICK_BUTTON:
                break;
        }
    }   //driverControllerButtonEvent

    /**
     * This method is called when an operator controller button event is detected.
     *
     * @param button specifies the button ID that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void operatorControllerButtonEvent(int button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "OperatorController: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcXboxController.BUTTON_A:
                if (robot.intake != null && robot.shooter != null && pressed)
                {
                    boolean autoAssistActive =
                        !(robot.autoPickupFromSource.isActive() || robot.autoPickupFromGround.isActive());
                    if (autoAssistActive)
                    {
                        if (altFunc)
                        {
                            // Intake from source with no vision.
                            robot.autoPickupFromSource.autoAssistPickup(false, false, null);
                        }
                        else
                        {
                            // Intake from ground with no vision.
                            robot.autoPickupFromGround.autoAssistPickup(false, null);
                        }
                    }
                    else
                    {
                        robot.autoAssistCancel();
                    }
                }
                break;

            case FrcXboxController.BUTTON_B:
                if (robot.intake != null && robot.shooter != null && pressed)
                {
                    boolean autoAssistActive = !robot.autoScoreNote.isActive();
                    if (autoAssistActive)
                    {
                        if (altFunc)
                        {
                            // Shoot at Amp with no vision.
                            robot.autoScoreNote.autoAssistScore(TargetType.Amp, false, false, false, null);
                            // robot.intake.autoEjectForward(RobotParams.Intake.ejectForwardPower, 0.0);
                        }
                        else
                        {
                            // Shoot at Speaker with vision.
                            robot.autoScoreNote.autoAssistScore(TargetType.Speaker, true, true, true, null);
                            // robot.intake.autoEjectForward(RobotParams.Intake.ejectForwardPower, 0.0);
                        }
                    }
                    else
                    {
                        robot.autoAssistCancel();
                    }
                }
                break;

            case FrcXboxController.BUTTON_X:
                if (robot.climber != null && pressed)
                {
                    robot.climber.extend();
                }
                break;

            case FrcXboxController.BUTTON_Y:
                if (robot.climber != null && pressed)
                {
                    robot.climber.climb();
                }
                break;

            case FrcXboxController.LEFT_BUMPER:
                altFunc = pressed;
                if (robot.shooter != null)
                {
                    robot.shooter.setManualOverrideEnabled(altFunc);
                }

                if (robot.climber != null)
                {
                    robot.climber.setManualOverrideEnabled(altFunc);
                }
                break;

            case FrcXboxController.RIGHT_BUMPER:
                break;

            case FrcXboxController.DPAD_UP:
                if (robot.shooter != null && pressed)
                {
                    if (altFunc)
                    {
                        robot.shooter.shooterMotor.presetVelocityUp(null);
                    }
                    else
                    {
                        robot.shooter.tiltMotor.presetPositionUp(null, RobotParams.Shooter.tiltPowerLimit);
                    }
                }
                break;

            case FrcXboxController.DPAD_DOWN:
                if (robot.shooter != null && pressed)
                {
                    if (altFunc)
                    {
                        robot.shooter.shooterMotor.presetVelocityDown(null);
                    }
                    else
                    {
                        robot.shooter.tiltMotor.presetPositionDown(null, RobotParams.Shooter.tiltPowerLimit);
                    }
                }
                break;

            case FrcXboxController.DPAD_LEFT:
                break;

            case FrcXboxController.DPAD_RIGHT:
                break;

            case FrcXboxController.BACK:
                break;

            case FrcXboxController.START:
                break;

            case FrcXboxController.LEFT_STICK_BUTTON:
                break;

            case FrcXboxController.RIGHT_STICK_BUTTON:
                break;
        }
    }   //operatorControllerButtonEvent

    /**
     * This method is called when a right driver stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void leftDriveStickButtonEvent(int button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "LeftDriveStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    }   //leftDriveStickButtonEvent

    /**
     * This method is called when a right driver stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void rightDriveStickButtonEvent(int button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "RightDriveStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.SIDEWINDER_TRIGGER:
                // Toggle between field or robot oriented driving.
                if (robot.robotDrive != null && pressed)
                {
                    if (robot.robotDrive.driveBase.getDriveOrientation() != DriveOrientation.FIELD)
                    {
                        robot.robotDrive.driveBase.setDriveOrientation(DriveOrientation.FIELD, true);
                    }
                    else
                    {
                        robot.robotDrive.driveBase.setDriveOrientation(DriveOrientation.ROBOT, false);
                    }
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                // Inverted drive only makes sense for robot oriented driving.
                if (robot.robotDrive != null &&
                    robot.robotDrive.driveBase.getDriveOrientation() == DriveOrientation.ROBOT)
                {
                    robot.robotDrive.driveBase.setDriveOrientation(
                        pressed? DriveOrientation.INVERTED: DriveOrientation.ROBOT, false);
                }
                break;
        }
    }   //rightDriveStickButtonEvent

    /**
     * This method is called when an operator stick button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void operatorStickButtonEvent(int button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "OperatorStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    }   //operatorStickButtonEvent

    /**
     * This method is called when a button panel button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void buttonPanelButtonEvent(int button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "ButtonPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_BUTTON_RED1:
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN1:
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE1:
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW1:
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE1:
                break;

            case FrcJoystick.PANEL_BUTTON_RED2:
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN2:
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE2:
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW2:
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE2:
                break;
        }
    }   //buttonPanelButtonEvent

    /**
     * This method is called when a switch panel button event is detected.
     *
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void switchPanelButtonEvent(int button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, ">>>>> button=%d, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "SwitchPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_SWITCH_WHITE1:
                break;

            case FrcJoystick.PANEL_SWITCH_RED1:
                break;

            case FrcJoystick.PANEL_SWITCH_GREEN1:
                break;

            case FrcJoystick.PANEL_SWITCH_BLUE1:
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW1:
                break;

            case FrcJoystick.PANEL_SWITCH_WHITE2:
                break;

            case FrcJoystick.PANEL_SWITCH_RED2:
                break;

            case FrcJoystick.PANEL_SWITCH_GREEN2:
                break;

            case FrcJoystick.PANEL_SWITCH_BLUE2:
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW2:
                break;
        }
    }   //switchPanelButtonEvent

}   //class FrcTeleOp
