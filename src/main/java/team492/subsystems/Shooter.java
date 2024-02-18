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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcIntake;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcCANFalcon;
import TrcFrcLib.frclib.FrcCANSparkMax;
import team492.RobotParams;

public class Shooter implements TrcExclusiveSubsystem
{
    private static final String moduleName = Shooter.class.getSimpleName();

    private final TrcIntake intake;
    private final TrcDbgTrace tracer;
    public final FrcCANFalcon shooterMotor;
    public final FrcCANSparkMax tilterMotor;
    private final TrcEvent shooterOnTargetEvent;
    private final TrcEvent tilterOnTargetEvent;
    private final TrcEvent shootCompletionEvent;
    private final TrcTimer timer;

    private String currOwner = null;
    private boolean manualOverride = false;
    private TrcEvent completionEvent = null;

    /**
     * Constructor: Create an instance of the object.
     */
    public Shooter(TrcIntake intake)
    {
        this.intake = intake;
        tracer = new TrcDbgTrace();

        shooterMotor = new FrcCANFalcon(moduleName + ".shooterMotor", RobotParams.Shooter.shooterCandId);
        shooterMotor.resetFactoryDefault();
        shooterMotor.setMotorInverted(RobotParams.Shooter.shooterMotorInverted);
        shooterMotor.setBrakeModeEnabled(false);
        shooterMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        shooterMotor.setPositionSensorScaleAndOffset(RobotParams.Shooter.shooterPosScale, 0.0);
        shooterMotor.setVelocityPidCoefficients(RobotParams.Shooter.shooterVelPidCoeff);

        tilterMotor = new FrcCANSparkMax(moduleName + ".tilterMotor", RobotParams.Shooter.tilterCanId, false, true);
        tilterMotor.resetFactoryDefault();
        tilterMotor.setMotorInverted(RobotParams.Shooter.tilterMotorInverted);
        tilterMotor.setBrakeModeEnabled(true);
        tilterMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        tilterMotor.setCurrentLimit(
            RobotParams.Shooter.tilterCurrentLimit, RobotParams.Shooter.tilterCurrentThreshold,
            RobotParams.Shooter.tilterCurrentThresholdTime);
        tilterMotor.enableLowerLimitSwitch(true);
        tilterMotor.enableUpperLimitSwitch(true);
        tilterMotor.setPositionSensorScaleAndOffset(
            RobotParams.Shooter.tilterPosScale, RobotParams.Shooter.tilterPosOffset);
        // We are using software position PID control for Tilter. So we just enable software PID before setting
        // PID coefficients.
        tilterMotor.setSoftwarePidEnabled(true);
        tilterMotor.setPositionPidCoefficients(RobotParams.Shooter.tilterPosPidCoeff);
        tilterMotor.setPositionPidPowerComp(this::getTilterGravityComp);

        shooterOnTargetEvent = new TrcEvent(moduleName + ".shooterOnTargetEvent");
        tilterOnTargetEvent = new TrcEvent(moduleName + ".tilterOnTargetEvent");
        shootCompletionEvent = new TrcEvent(moduleName + ".shootCompletionEvent");
        timer = new TrcTimer(moduleName + ".timer");
    }   //Shooter

    //
    // Shooter subsystem methods.
    //

    /**
     * This method is called when the shooter operation is finished or canceled.
     *
     * @param completed specifies true if the operation is completed, false if canceled.
     */
    private void finish(boolean completed)
    {
        timer.cancel();
        shooterMotor.stop();
        tilterMotor.stop();

        if (currOwner != null)
        {
            releaseExclusiveAccess(currOwner);
            currOwner = null;
        }

        if (completionEvent != null)
        {
            if (completed)
            {
                completionEvent.signal();
            }
            else
            {
                completionEvent.cancel();
            }
            completionEvent = null;
        }
    }   //finish

    /**
     * This method cancel a pending shooter operation if any.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     */
    private void cancel(String owner)
    {
        tracer.traceInfo(moduleName, "owner=" + owner);
        if (validateOwnership(owner))
        {
            finish(false);
        }
    }   //cancel

    /**
     * This method cancel a pending shooter operation if any.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method prepares the shooter subsystem for shooting. It sets the shooter velocity and the tilter angle.
     * This method is asynchronous. When both shooter velocity and tilter position have reached target, it will shoot
     * and signal an event if provided.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param velocity specifies the shooter velocity in revolutions per second.
     * @param tiltAngle specifies the absolute tilter angle in degrees.
     * @param event specifies an event to signal when both reached target, can be null if not provided.
     * @param timeout specifies maximum timeout period, can be zero if no timeout.
     */
    public void aimAndShoot(String owner, double velocity, double tiltAngle, TrcEvent event, double timeout)
    {
        tracer.traceInfo(
            moduleName,
            "owner=" + owner +
            ", vel=" + velocity +
            ", angle=" + tiltAngle +
            ", event=" + event +
            ", timeout=" + timeout);
        // Caller specifies an owner but has not acquired ownership, let's acquire ownership on its behalf.
        if (owner != null && !hasOwnership(owner) && acquireExclusiveAccess(owner))
        {
            currOwner = owner;
        }

        if (validateOwnership(owner))
        {
            this.completionEvent = event;

            shooterOnTargetEvent.clear();
            shooterOnTargetEvent.setCallback(this::onTarget, null);
            shooterMotor.setVelocity(0.0, velocity, 0.0, shooterOnTargetEvent);

            tilterOnTargetEvent.clear();
            tilterOnTargetEvent.setCallback(this::onTarget, null);
            tilterMotor.setPosition(0.0, tiltAngle, true, RobotParams.Shooter.tilterPowerLimit, tilterOnTargetEvent);

            if (timeout > 0.0)
            {
                timer.set(timeout, this::timedOut, null);
            }
        }
    }   //aimAndShoot

    /**
     * This method is called when the shooter has reached target velocity or tilter has reached target position.
     *
     * @param context not used.
     */
    private void onTarget(Object context)
    {
        tracer.traceInfo(moduleName, "shooterEvent=" + shooterOnTargetEvent + ", tilterEvent=" + tilterOnTargetEvent);
        if (shooterOnTargetEvent.isSignaled() && tilterOnTargetEvent.isSignaled())
        {
            // If both shooter velocity and tilter position have reached target, shoot.
            shootCompletionEvent.clear();
            shootCompletionEvent.setCallback(this::shootCompleted, null);
            intake.autoAssistEjectForward(0.0, RobotParams.Intake.ejectForwardPower, 0.0, shootCompletionEvent, 0.0);
        }
    }   //onTarget

    /**
     * This method is called when the object has been ejected from the shooter.
     *
     * @param context not used.
     */
    private void shootCompleted(Object context)
    {
        tracer.traceInfo(moduleName, "Shoot completed.");
        finish(true);
    }   //shootCompleted

    /**
     * This method is called if the shooter operation has timed out.
     *
     * @param context not used.
     */
    private void timedOut(Object context)
    {
        tracer.traceInfo(moduleName, "Timed out.");
        finish(false);
    }   //timedOut

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
     * This method is called by PID control to determine the power required to compensate for gravity in essence
     * making the tilter gravity neutral (i.e. hold its position, aka feedforward).
     *
     * @param currPower specifies the current tilter power (not used).
     * @return gravity compensation power.
     */
    private double getTilterGravityComp(double currPower)
    {
        double gravityComp = RobotParams.Shooter.tilterMaxHoldingPower * Math.cos(Math.toRadians(getTilterAngle()));
        tracer.traceDebug(moduleName, "gravityComp=" + gravityComp);
        return gravityComp;
    }   //getTilterGravityComp

    //
    // Shooter motor methods.
    //

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
     * This method sets the shooter velocity.
     *
     * @param velocity specifies the shooter velocity in revolutions per second.
     */
    public void setShooterVelocity(double velocity)
    {
        shooterMotor.setVelocity(null, 0.0, velocity, 0.0, null);
    }   //setShooterVelocity

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
     * This method returns the current absolute tilter angle from horizontal.
     *
     * @return current tilter angle in degrees.
     */
    public double getTilterAngle()
    {
        return tilterMotor.getPosition();
    }   //getTilterAngle

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
     * This method returns the current applied tilter power duty cycle (in the range of -1 to 1).
     *
     * @return current tilter power.
     */
    public double getTilterPower()
    {
        return tilterMotor.getPower();
    }   //getTilterPower

    /**
     * This methods moves the tilter up and down with the specified power. It is typically used by TeleOp to control
     * the tilter by a joystick value. The tilter movement is PID controlled when manual override is not enabled.
     *
     * @param power specifies the power duty cycle used to move the tilter (in the range of -1 to 1).
     */
    public void setTilterPower(double power)
    {
        if (manualOverride)
        {
            tilterMotor.setPower(null, 0.0, power, 0.0, null);;
        }
        else
        {
            tilterMotor.setPidPower(null, power, RobotParams.Shooter.tilterMinPos, RobotParams.Shooter.tilterMaxPos, true);
        }
    }   //setTilterPower

    /**
     * This method checks if the tilter's lower limit switch is active.
     *
     * @return true if active, false otherwise.
     */
    public boolean tilterLowerLimitSwitchActive()
    {
        return tilterMotor.isLowerLimitSwitchActive();
    }   //tilterLowerLimitSwitchActive

    /**
     * This method checks if the tilter's upper limit switch is active.
     *
     * @return true if active, false otherwise.
     */
    public boolean tilterUpperLimitSwitchActive()
    {
        return tilterMotor.isUpperLimitSwitchActive();
    }   //tilterUpperLimitSwitchActive

}   //class Shooter
