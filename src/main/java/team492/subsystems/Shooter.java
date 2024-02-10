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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492.subsystems;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcCANFalcon;
import team492.RobotParams;

public class Shooter
{
    private static final String moduleName = Shooter.class.getSimpleName();

    public final FrcCANFalcon shooterMotor;
    public final FrcCANFalcon tilterMotor;  //CodeReview: not a Falcon, it's a SparkMax brushed motor.
    private final TrcTaskMgr.TaskObject shooterTaskObj;
    private boolean manualOverride = false;
    private TrcEvent completionEvent = null;

    /**
     * Constructor: Create an instance of the object.
     */
    public Shooter()
    {
        shooterMotor = new FrcCANFalcon(moduleName + ".shooterMotor", RobotParams.Shooter.shooterCandId);
        shooterMotor.resetFactoryDefault();
        shooterMotor.setMotorInverted(RobotParams.Shooter.shooterMotorInverted);
        shooterMotor.setBrakeModeEnabled(false);
        shooterMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        shooterMotor.enableMotionProfile(0.0, RobotParams.Shooter.shooterAcceleration, 0.0);
        shooterMotor.setPositionSensorScaleAndOffset(RobotParams.Shooter.shooterPosScale, 0.0);
        shooterMotor.setVelocityPidCoefficients(RobotParams.Shooter.shooterVelPidCoeff);

        tilterMotor = new FrcCANFalcon(moduleName + ".tilterMotor", RobotParams.Shooter.tilterCanId);
        tilterMotor.resetFactoryDefault();
        tilterMotor.setMotorInverted(RobotParams.Shooter.tilterMotorInverted);
        tilterMotor.setBrakeModeEnabled(true);
        tilterMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        tilterMotor.enableMotionProfile(
            RobotParams.Shooter.tilterVelocity, RobotParams.Shooter.tilterAcceleration, 0.0);
        tilterMotor.setCurrentLimit(
            RobotParams.Shooter.tilterCurrentLimit, RobotParams.Shooter.tilterCurrentThreshold,
            RobotParams.Shooter.tilterCurrentThresholdTime);
        tilterMotor.enableLowerLimitSwitch(true);
        tilterMotor.enableUpperLimitSwitch(true);
        tilterMotor.setPositionSensorScaleAndOffset(
            RobotParams.Shooter.tilterPosScale, RobotParams.Shooter.tilterPosOffset);
        tilterMotor.setSoftPositionLimits(RobotParams.Shooter.tilterMinPos, RobotParams.Shooter.tilterMaxPos, false);
        tilterMotor.setPositionPidCoefficients(RobotParams.Shooter.tilterPosPidCoeff);

        // Sync absolute encoder to motor encoder DONE, not sure if this is right though
        // CodeReview: Need to talk to mechanical to find out what absolute encoder they have.
        // Also, mechanical said the tilter motor is not a Falcon, it's a SparkMax brushed motor.
        tilterMotor.setPosition(tilterMotor.getEncoderRawPosition());

        shooterTaskObj = TrcTaskMgr.createTask("ShooterTask", this::shooterTask);
    }   //Shooter

    //
    // Shooter subsystem methods.
    //

    /**
     * This method prepares the shooter subsystem for shooting. It sets the shooter velocity and the tilter angle.
     * This method is asynchronous. It starts a task to monitor the shooter velocity and tilter angle. When both
     * reached target, it will signal an event if provided.
     *
     * @param velocity specifies the shooter velocity in revolutions per second.
     * @param tiltAngle specifies the absolute tilter angle in degrees.
     * @param event specifies an event to signal when both reached target, can be null if not provided.
     */
    public void prepForShooting(double velocity, double tiltAngle, TrcEvent event)
    {
        // TODO: Implement this.
        // Set up velocity, tiltAngle and event.
        // Set shooter velocity.
        // Set tilter angle.
        // Do we need timeout?
        // Enable task to monitor velocity and tiltAngle.
    }   //prepForShooting

    /**
     * This method enables the shooter task that monitors if the shooter velocity as well as the tilter angle are on
     * target.
     *
     * @param enabled specifies true to enable shooter task, false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            shooterTaskObj.registerTask(TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        }
        else
        {
            shooterTaskObj.unregisterTask();
        }
    }   //setTaskEnabled

    /**
     * This method enables/disables manual override for setTilterPower. When manual override is not enabled,
     * setTilterPower will operate the tilter by PID control which means it will slow down the approach when
     * it's near the upper or lower angle limits. When manul override is enabled, it will simply applied the
     * specified power as is.
     *
     * @param enabled specifies true to enable manual override, false to disable.
     */
    public void setManualOverrideEnabled(boolean enabled)
    {
        manualOverride = enabled;
    }   //setManualOverrideEnabled

    /**
     * This method checks if manual override is enabled.
     *
     * @return true if manual override is enabled, false if disabled.
     */
    public boolean isManualOverrideEnabled()
    {
        return manualOverride;
    }   //isManualOverrideEnabled

    /**
     * This methods is called periodically to run the shooter task.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void shooterTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        // TODO: Implement this.
        // If tilter has not reached target, setPosition with GravityComp.
        // else if shooter has reached target, signal event and disable task.
    }   //shooterTask

    /**
     * This method is called by PID control to determine the power required to compensate for gravity in essence
     * making the tilter gravity neutral (i.e. hold its position, aka feedforward).
     *
     * @return gravity compensation power.
     */
    private double getTilterGravityComp()
    {
        // TODO: Implement this.
        return 0; // theoretically, it's gravity neutral
    }   //getTilterGravityComp

    //
    // Shooter motor methods.
    //

    /**
     * This method sets the shooter velocity.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param velocity specifies the shooter velocity in revolutions per second.
     */
    public void setShooterVelocity(String owner, double velocity)
    {
        shooterMotor.setVelocity(owner, 0.0, velocity, 0.0, null);
    }   //setShooterVelocity

    /**
     * This method sets the shooter velocity.
     *
     * @param velocity specifies the shooter velocity in revolutions per second.
     */
    public void setShooterVelocity(double velocity)
    {
        shooterMotor.setVelocity(null, 0.0, velocity, 0.0, null);
    }   //setShooterVelocity

    /**
     * This method returns the current shooter velocity.
     *
     * @return current shooter velocity in revolutions per second.
     */
    public double getShooterVelocity()
    {
        return shooterMotor.getVelocity();
    }   //getShooterVelocity

    /**
     * This method stops the shooter. Use this method instead of setting shooter velocity to zero because the shooter
     * will coast to a stop instead of stopping abruptly.
     */
    public void stopShooter()
    {
        shooterMotor.stop();
    }   //stopShooter

    //
    // Tilter motor methods.
    //

    /**
     * This method sets the tilter angle.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param angle specifies the tilter absolute angle from horizontal in degrees (horizontal is 0-degree).
     * @param completionEvent specifies the event to signal when tilter reached target angle, can be null if not
     *        provided.
     * @param timeout specifies timeout in seconds in case PID control cannot reach target.
     */
    public void setTilterAngle(String owner, double angle, TrcEvent completionEvent, double timeout)
    {
        tilterMotor.setPosition(
            owner, 0.0, angle, true, RobotParams.Shooter.tilterPowerLimit, completionEvent, timeout);
    }   //setTilterAngle

    /**
     * This method sets the tilter angle.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param angle specifies the tilter absolute angle from horizontal in degrees (horizontal is 0-degree).
     */
    public void setTilterAngle(String owner, double angle)
    {
        tilterMotor.setPosition(
            owner, 0.0, angle, true, RobotParams.Shooter.tilterPowerLimit, null, 0.0);
    }   //setTilterAngle

    /**
     * This method sets the tilter angle.
     *
     * @param angle specifies the tilter absolute angle from horizontal in degrees (horizontal is 0-degree).
     */
    public void setTilterAngle(double angle)
    {
        tilterMotor.setPosition(
            null, 0.0, angle, true, RobotParams.Shooter.tilterPowerLimit, null, 0.0);
    }   //setTilterAngle

    /**
     * This method returns the current absolute tilter angle from horizontal.
     *
     * @return current tilter angle in degrees.
     */
    public double getTilterAngle()
    {
        return tilterMotor.getPosition();
    }   //getTilterAngle

    /**
     * This methods moves the tilter up and down with the specified power. It is typically used by TeleOp to control
     * the tilter by a joystick value. The tilter movement is PID controlled when manual override is not enabled.
     *
     * @param power specifies the power duty cycle used to move the tilter (in the range of -1 to 1).
     */
    public void setTilterPower(double power)
    {
        // TODO: Implement this.
        // if manualOverride call motor.setPower else call motor.setPidPower
    }   //setTilterPower

    /**
     * This method returns the current applied tilter power duty cycle (in the range of -1 to 1).
     *
     * @return current tilter power.
     */
    public double getTilterPower()
    {
        // TODO: Implement this.
        return 0.0;
    }   //getTilterPower

    /**
     * This method checks if the tilter's lower limit switch is active.
     *
     * @return true if active, false otherwise.
     */
    public boolean tilterLowerLimitSwitchActive()
    {
        // TODO: Implement this.
        return false;
    }   //tilterLowerLimitSwitchActive

    /**
     * This method checks if the tilter's upper limit switch is active.
     *
     * @return true if active, false otherwise.
     */
    public boolean tilterUpperLimitSwitchActive()
    {
        // TODO: Implement this.
        return false;
    }   //tilterUpperLimitSwitchActive

}   //class Shooter
