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

import java.util.Locale;

import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcDriveBase.DriveOrientation;
import TrcCommonLib.trclib.TrcRobot.RunMode;
import TrcFrcLib.frclib.FrcCANSparkMax;
import TrcFrcLib.frclib.FrcLogitechJoystick;
import TrcFrcLib.frclib.FrcPanelButtons;
import TrcFrcLib.frclib.FrcPhotonVision;
import TrcFrcLib.frclib.FrcSideWinderJoystick;
import TrcFrcLib.frclib.FrcXboxController;
import team492.subsystems.Shooter;

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
    protected boolean driverAltFunc = false;
    private boolean driverTracking = false;
    private boolean operatorTracking = false;
    private boolean prevTrackingModeOn = false;
    protected boolean operatorAltFunc = false;
    private boolean subsystemStatusOn = true;
    // DriveBase subsystem.
    private TrcPidController trackingPidCtrl;
    private double driveSpeedScale = RobotParams.DRIVE_NORMAL_SCALE;
    private double turnSpeedScale = RobotParams.TURN_NORMAL_SCALE;
    // private double[] prevDriveInputs = null;
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
        robot.setDriveOrientation(DriveOrientation.FIELD, true);
        // trackingPidCtrl should have same PID coefficients as the PurePursuit turn PID controller.
        trackingPidCtrl = robot.robotDrive == null? null:
            new TrcPidController(
                "trackingPidCtrl", robot.robotDrive.purePursuitDrive.getTurnPidCtrl().getPidCoefficients(), null);
        trackingPidCtrl.setAbsoluteSetPoint(true);
        trackingPidCtrl.setInverted(true);

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
            FrcPhotonVision.DetectedObject aprilTagObj = null;
            //
            // Update Vision LEDs.
            //
            if (robot.photonVisionFront != null)
            {
                aprilTagObj = robot.photonVisionFront.getBestDetectedAprilTag(4, 7, 5, 6, 3, 8);
                // if (aprilTagObj != null)
                // {
                //     robot.relocalize(aprilTagObj);
                // }
            }

            if (robot.photonVisionBack != null)
            {
                robot.photonVisionBack.getBestDetectedObject();
            }

            if (controlsEnabled)
            {
                //
                // DriveBase operation.
                //
                if (robot.robotDrive != null)
                {
                    double[] driveInputs = robot.robotDrive.getDriveInputs(
                        RobotParams.ROBOT_DRIVE_MODE, true, driveSpeedScale, turnSpeedScale);
                    double rotPower = driveInputs[2];
                    int aprilTagId = -1;
                    Double shooterVel = null, tiltAngle = null;

                    if (robot.shooter != null)
                    {
                        boolean trackingModeOn = driverTracking || operatorTracking;
                        if (trackingModeOn)
                        {
                            if (aprilTagObj != null)
                            {
                                TrcPose2D aprilTagPose = robot.aimShooterAtAprilTag(aprilTagObj);
                                rotPower = trackingPidCtrl.getOutput(aprilTagPose.angle, 0.0);
                                // robot.globalTracer.traceInfo(moduleName, "aprilTagAngle=" + aprilTagPose.angle + ", rotPower=" + rotPower);

                            }
                        }
                        else if (prevTrackingModeOn)
                        {
                            robot.turtle();
                        }
                        prevTrackingModeOn = trackingModeOn;
                    }

                    // if (!Arrays.equals(driveInputs, prevDriveInputs))
                    // {
                        if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                        {
                            double gyroAngle = robot.robotDrive.driveBase.getDriveGyroAngle();
                            robot.robotDrive.driveBase.holonomicDrive(
                                null, driveInputs[0], driveInputs[1], rotPower, gyroAngle);
                            if (subsystemStatusOn)
                            {
                                String s = String.format(
                                    Locale.US, "Holonomic: x=%.3f, y=%.3f, rot=%.3f, angle=%.3f",
                                    driveInputs[0], driveInputs[1], rotPower, gyroAngle);
                                if (aprilTagId != -1)
                                {
                                    s += String.format(
                                        ", Id=%d, shooterVel=%.1f, tilt=%.1f", aprilTagId, shooterVel, tiltAngle);
                                }
                                robot.dashboard.displayPrintf(lineNum++, s);
                            }
                        }
                        else if (RobotParams.Preferences.useTankDrive)
                        {
                            robot.robotDrive.driveBase.tankDrive(driveInputs[0], driveInputs[1]);
                            if (subsystemStatusOn)
                            {
                                robot.dashboard.displayPrintf(
                                    lineNum++, "Tank: left=%.3f, right=%.3f, rot=%.3f",
                                    driveInputs[0], driveInputs[1], driveInputs[2]);
                            }
                        }
                        else
                        {
                            robot.robotDrive.driveBase.arcadeDrive(driveInputs[1], driveInputs[2]);
                            if (subsystemStatusOn)
                            {
                                robot.dashboard.displayPrintf(
                                    lineNum++, "Arcade: x=%.3f, y=%.3f, rot=%.3f",
                                    driveInputs[0], driveInputs[1], driveInputs[2]);
                            }
                        }
                    // }
                    // else if (subsystemStatusOn)
                    // {
                    //     lineNum++;
                    // }
                    // prevDriveInputs = driveInputs;
                    if (subsystemStatusOn)
                    {
                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s, Orient=%s, GyroAssist=%s",
                            robot.robotDrive.driveBase.getFieldPosition(),
                            robot.robotDrive.driveBase.getDriveOrientation(),
                            robot.robotDrive.driveBase.isGyroAssistEnabled());
                    }
                }
                //
                // Analog control of subsystem is done here if necessary.
                //
                if (RobotParams.Preferences.useSubsystems)
                {
                    if (robot.intake != null && subsystemStatusOn)
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

                        if (subsystemStatusOn)
                        {
                            String msg = String.format(
                                Locale.US, "Shooter: vel=%.0f/%.0f, preset=%.0f, inc=%.0f",
                                shooterVel, robot.shooter.getShooterVelocity(), robot.shooterVelocity.getValue(),
                                robot.shooterVelocity.getIncrement());
                            if (robot.deflector != null)
                            {
                                msg += ", deflector=" + robot.deflector.isExtended();
                            }
                            robot.dashboard.displayPrintf(lineNum++, msg);
                        }

                        double tiltPower = robot.operatorController.getLeftYWithDeadband(true);
                        // Only set tilt power if it is different from previous value.
                        if (prevTiltPower != tiltPower)
                        {
                            robot.shooter.setTiltPower(tiltPower);
                            prevTiltPower = tiltPower;
                        }

                        if (subsystemStatusOn)
                        {
                            robot.dashboard.displayPrintf(
                                lineNum++,
                                "Tilt: power=%.2f/%.2f, angle=%.2f/%.2f/%f, inc=%.0f, limits=%s/%s" +
                                ", yaw/pitch/roll=%.2f/%.2f/%.2f",
                                tiltPower, robot.shooter.getTiltPower(), robot.shooter.getTiltAngle(),
                                robot.shooter.tiltMotor.getPidTarget(), robot.shooter.tiltMotor.getMotorPosition(),
                                robot.shooterTiltAngle.getIncrement(), robot.shooter.tiltLowerLimitSwitchActive(),
                                robot.shooter.tiltUpperLimitSwitchActive(),
                                Shooter.getTilterYaw(), Shooter.getTilterPitch(), Shooter.getTilterRoll());
                        }
                    }

                    if (robot.climber != null)
                    {
                        double climbPower = robot.operatorController.getRightYWithDeadband(true);
                        if (prevClimbPower != climbPower)
                        {
                            robot.climber.setClimbPower(climbPower);
                            prevClimbPower = climbPower;
                        }

                        if (subsystemStatusOn)
                        {
                            robot.dashboard.displayPrintf(
                                lineNum++, "Climber: power=%.2f/%.2f, current=%.3f, pos=%.2f/%.2f/%f, limits=%s/%s",
                                climbPower, robot.climber.climberMotor.getPower(), robot.climber.climberMotor.getCurrent(),
                                robot.climber.getPosition(), robot.climber.climberMotor.getPidTarget(),
                                robot.climber.climberMotor.getMotorPosition(),
                                robot.climber.climberMotor.isLowerLimitSwitchActive(),
                                robot.climber.climberMotor.isUpperLimitSwitchActive());
                        }
                    }

                    if (robot.ultrasonicSensor != null && subsystemStatusOn)
                    {
                        robot.dashboard.displayPrintf(
                            lineNum++, "Ultrasonic: distance=%.3f", robot.getUltrasonciDistance());
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
     * @param buttonValue specifies the button enum value that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void driverControllerButtonEvent(int buttonValue, boolean pressed)
    {
        FrcXboxController.Button button = FrcXboxController.Button.values()[buttonValue];

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=%s, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "DriverController: button %s %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case BUTTON_A:
                // Toggle between field or robot oriented driving.
                if (robot.robotDrive != null && pressed)
                {
                    if (robot.robotDrive.driveBase.getDriveOrientation() != DriveOrientation.FIELD)
                    {
                        robot.setDriveOrientation(DriveOrientation.FIELD, true);
                    }
                    else
                    {
                        robot.setDriveOrientation(DriveOrientation.ROBOT, false);
                    }
                }
                break;

            case BUTTON_B:
                // Turtle mode.
                if (pressed)
                {
                    robot.turtle();
                }
                break;

            case BUTTON_X:
                // AutoIntake from ground with Vision, hold AltFunc for no vision.
                if (robot.intake != null && pressed)
                {
                    boolean active = !robot.autoPickupFromGround.isActive();
                    if (active)
                    {
                        // Press and hold altFunc for manual intake (no vision).
                        robot.autoPickupFromGround.autoAssistPickup(driverAltFunc, false, null);
                    }
                    else
                    {
                        robot.autoAssistCancel();
                    }
                }
                break;

            case BUTTON_Y:
                driverTracking = pressed;
                break;

            case LEFT_BUMPER:
                driverAltFunc = pressed;
                if (!driverAltFunc && robot.shooter != null)
                {
                    robot.shooter.stopShooter();
                }
                break;

            case RIGHT_BUMPER:
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

            case DPAD_RIGHT:
                // Manual shoot.
                if (robot.robotDrive != null && robot.intake != null)
                {
                    if (pressed)
                    {
                        robot.robotDrive.setXModeEnabled(moduleName, true);
                        robot.intake.autoEjectForward(RobotParams.Intake.ejectForwardPower, 0.0);
                    }
                    else
                    {
                        robot.robotDrive.setXModeEnabled(moduleName, false);
                    }
                }
                break;

            case DPAD_LEFT:

                // Aim at Amp.
                if (robot.intake != null && robot.shooter != null && pressed)
                {
                    boolean active = !robot.shooter.isActive();
                    if (active)
                    {
                        robot.shooter.aimShooter(
                            RobotParams.Shooter.shooterAmpVelocity,
                            RobotParams.Shooter.tiltAmpAngle, 0.0);
                        robot.deflector.extend();
                        // robot.shooter.setShooterVelocity(RobotParams.Shooter.shooterAmpVelocity);
                        // robot.shooter.setTiltAngle(RobotParams.Shooter.tiltAmpAngle);
                    }
                    else
                    {
                        robot.shooter.cancel();
                        robot.deflector.retract();
                    }
                }


                break;

            case BACK:
                if (pressed)
                {
                    robot.autoAssistCancel();
                }
                break;

            case START:
                if (pressed)
                {
                    subsystemStatusOn = !subsystemStatusOn;
                }
                break;

            case LEFT_STICK_BUTTON:
            case RIGHT_STICK_BUTTON:
            default:
                break;
        }
    }   //driverControllerButtonEvent

    /**
     * This method is called when an operator controller button event is detected.
     *
     * @param buttonValue specifies the button enum value that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void operatorControllerButtonEvent(int buttonValue, boolean pressed)
    {
        FrcXboxController.Button button = FrcXboxController.Button.values()[buttonValue];

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=%s, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "OperatorController: button %s %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case BUTTON_A:
                // Aim at Speaker up-close.
                if (robot.shooter != null && pressed)
                {
                    boolean active = !robot.shooter.isActive();
                    if (active)
                    {
                        robot.shooter.aimShooter(
                            RobotParams.Shooter.shooterSpeakerCloseVelocity,
                            RobotParams.Shooter.tiltSpeakerCloseAngle, 0.0);
                        // robot.shooter.setShooterVelocity(RobotParams.Shooter.shooterSpeakerCloseVelocity);
                        // robot.shooter.setTiltAngle(RobotParams.Shooter.tiltSpeakerCloseAngle);
                    }
                    else
                    {
                        robot.shooter.cancel();
                    }
                }
                break;

            case BUTTON_B:
                // Manual shoot.
                if (robot.robotDrive != null && robot.intake != null)
                {
                    if (pressed)
                    {
                        robot.robotDrive.setXModeEnabled(moduleName, true);
                        robot.intake.autoEjectForward(RobotParams.Intake.ejectForwardPower, 0.0);
                    }
                    else
                    {
                        robot.robotDrive.setXModeEnabled(moduleName, false);
                    }
                }
                break;

            case BUTTON_X:
                // Aim at Amp.
                if (robot.intake != null && robot.shooter != null && pressed)
                {
                    boolean active = !robot.shooter.isActive();
                    if (active)
                    {
                        robot.shooter.aimShooter(
                            RobotParams.Shooter.shooterAmpVelocity, RobotParams.Shooter.tiltAmpAngle, 0.0);
                        robot.deflector.extend();
                    }
                    else
                    {
                        robot.shooter.cancel();
                        robot.deflector.retract();
                    }
                }
                break;

            case BUTTON_Y:
                operatorTracking = pressed;
                // // AutoShoot at Speaker with Vision, hold AltFunc for no vision.
                // if (robot.intake != null && robot.shooter != null && pressed)
                // {
                //     boolean active = !robot.autoScoreNote.isActive();
                //     if (active)
                //     {
                //         // Press and hold altFunc for manual shooting (no vision).
                //         robot.autoScoreNote.autoAssistScore(TargetType.Speaker, !operatorAltFunc);
                //     }
                //     else
                //     {
                //         robot.autoAssistCancel();
                //     }
                // }
                break;

            case LEFT_BUMPER:
                operatorAltFunc = pressed;
                if (robot.shooter != null)
                {
                    robot.shooter.setManualOverrideEnabled(operatorAltFunc);
                }

                if (robot.climber != null)
                {
                    robot.climber.setManualOverrideEnabled(operatorAltFunc);
                }
                break;

            case RIGHT_BUMPER:
                //Turtle mode
                if (pressed)
                {
                    robot.turtle();
                }
                break;

            case DPAD_UP:
                // AutoIntake from ground with Vision.
                if (robot.intake != null && pressed)
                {
                    boolean active = !robot.autoPickupFromGround.isActive();
                    if (active)
                    {
                        robot.autoPickupFromGround.autoAssistPickup(true, false, null);
                    }
                    else
                    {
                        robot.autoAssistCancel();
                    }
                }
                break;

            case DPAD_DOWN:
                // AutoIntake from ground with no Vision (manual pickup), hold AltFunc for ReverseIntake.
                if (robot.intake != null && pressed)
                {
                    if (operatorAltFunc)
                    {
                        robot.intake.autoIntakeReverse(RobotParams.Intake.intakePower, 0.0, 0.0);
                    }
                    else
                    {
                        boolean active = !robot.autoPickupFromGround.isActive();
                        if (active)
                        {
                            robot.autoPickupFromGround.autoAssistPickup(false, false, null);
                        }
                        else
                        {
                            robot.autoAssistCancel();
                        }
                    }
                }
                break;

            case DPAD_LEFT:
                break;

            case DPAD_RIGHT:
                if(pressed){
                    Shooter.clearTilterFaults();    
                }
                
                break;

            case BACK:
                if (robot.climber != null && pressed)
                {
                    robot.climber.zeroCalibrate();
                }
                break;

            case START:
                if (robot.shooter != null && pressed)
                {
                    // If the Tilter hit something hard and caused the pulley to skip, the absolute encoder will be
                    // off. This provides an emergency way to quickly resync the encoder by using manual override to
                    // drive the Tilter to the lower limit and set the encoder postion as the soft zero. Then we set
                    // the encoder zeroOffset to zero. Note: this hack is volatile, meaning once the robot power is
                    // turned off, we lose this setting. The idea is that this hack is for emergency resync so that
                    // we can continue the match uninterrupted. We should always do the proper encoder calibration
                    // once we are back in the pit.
                    ((FrcCANSparkMax) robot.shooter.tiltMotor).resetMotorPosition(false);
                    robot.shooter.tiltMotor.setPositionSensorScaleAndOffset(
                        RobotParams.Shooter.tiltPosScale, RobotParams.Shooter.tiltPosOffset, 0.0);
                }
                break;

            case LEFT_STICK_BUTTON:
            case RIGHT_STICK_BUTTON:
            default:
                break;
        }
    }   //operatorControllerButtonEvent

    /**
     * This method is called when a right driver stick button event is detected.
     *
     * @param buttonValue specifies the button enum value that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void leftDriveStickButtonEvent(int buttonValue, boolean pressed)
    {
        FrcLogitechJoystick.Button button = FrcLogitechJoystick.Button.values()[buttonValue];

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=%s, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "LeftDriveStick: button %s %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            default:
                break;
        }
    }   //leftDriveStickButtonEvent

    /**
     * This method is called when a right driver stick button event is detected.
     *
     * @param buttonValue specifies the button enum value that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void rightDriveStickButtonEvent(int buttonValue, boolean pressed)
    {
        FrcSideWinderJoystick.Button button = FrcSideWinderJoystick.Button.values()[buttonValue];

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=%s, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "RightDriveStick: button %s %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            default:
                break;
        }
    }   //rightDriveStickButtonEvent

    /**
     * This method is called when an operator stick button event is detected.
     *
     * @param buttonValue specifies the button enum value that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void operatorStickButtonEvent(int buttonValue, boolean pressed)
    {
        FrcLogitechJoystick.Button button = FrcLogitechJoystick.Button.values()[buttonValue];

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=%s, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "OperatorStick: button %s %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            default:
                break;
        }
    }   //operatorStickButtonEvent

    /**
     * This method is called when a button panel button event is detected.
     *
     * @param buttonValue specifies the button enum value that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void buttonPanelButtonEvent(int buttonValue, boolean pressed)
    {
        FrcPanelButtons.Button button = FrcPanelButtons.Button.values()[buttonValue];

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=%s, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "ButtonPanel: button %s %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            default:
                break;
        }
    }   //buttonPanelButtonEvent

    /**
     * This method is called when a switch panel button event is detected.
     *
     * @param buttonValue specifies the button enum value that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void switchPanelButtonEvent(int buttonValue, boolean pressed)
    {
        FrcPanelButtons.Button button = FrcPanelButtons.Button.values()[buttonValue];

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=%s, pressed=%s", button, pressed);
        }

        robot.dashboard.displayPrintf(
            8, "SwitchPanel: button %s %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            default:
                break;
        }
    }   //switchPanelButtonEvent

}   //class FrcTeleOp
