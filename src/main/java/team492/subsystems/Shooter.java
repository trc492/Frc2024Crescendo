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
    public final FrcCANFalcon tilterMotor;
    private TrcTaskMgr.TaskObject shooterTaskObj;
    private boolean manualOverride = false;
    private TrcEvent completionEvent = null;

    public Shooter()
    {
        shooterMotor = new FrcCANFalcon(moduleName + ".shooterMotor", RobotParams.Shooter.shooterCandId);
        shooterMotor.resetFactoryDefault();
        shooterMotor.setMotorInverted(RobotParams.Shooter.shooterMotorInverted);
        shooterMotor.setBrakeModeEnabled(false);
        shooterMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        shooterMotor.setPositionSensorScaleAndOffset(RobotParams.Shooter.shooterPosScale, 0.0);
        // Configure shooter PID.

        tilterMotor = new FrcCANFalcon(moduleName + ".tilterMotor", RobotParams.Shooter.tilterCanId);
        tilterMotor.resetFactoryDefault();
        tilterMotor.setMotorInverted(RobotParams.Shooter.tilterMotorInverted);
        tilterMotor.setBrakeModeEnabled(true);
        tilterMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        // Configure current limit.
        tilterMotor.enableLowerLimitSwitch(true);
        tilterMotor.enableUpperLimitSwitch(true);
        tilterMotor.setPositionSensorScaleAndOffset(
            RobotParams.Shooter.tilterPosScale, RobotParams.Shooter.tilterPosOffset);
        // Configure tilter PID.
        // Configure soft position limits?
        // Configure Motion Magic.
        // Sync absolute encoder to motor encoder

        shooterTaskObj = TrcTaskMgr.createTask("ShooterTask", this::shooterTask);
    }

    //
    // Shooter subsystem methods.
    //

    public void prepForShooting(double velocity, double tiltAngle, TrcEvent event)
    {
        // Set up velocity, tiltAngle and event.
        // Set shooter velocity.
        // Set tilter angle.
        // Enable task to monitor velocity and tiltAngle.
    }

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
    }

    public void setManualOverrideEnabled(boolean enabled)
    {
        manualOverride = enabled;
        // if (!enabled)
        // {
        //     pitchTicksTarget = (int) pitchMotor.getPosition();
        // }
    }

    public boolean isManualOverrideEnabled()
    {
        return manualOverride;
    }

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
        // If tilter and shooter reached target, signal event, disable task.
    }

    private double getTilterGravityComp()
    {
        return 0; // theoretically, it's gravity neutral
    }

    //
    // Shooter motor methods.
    //

    public void setShooterVelocity(String owner, double velocity)
    {
        shooterMotor.setVelocity(owner, 0.0, velocity, 0.0, null);
    }

    public void setShooterVelocity(double velocity)
    {
        shooterMotor.setVelocity(null, 0.0, velocity, 0.0, null);
    }

    public double getShooterVelocity()
    {
        return shooterMotor.getVelocity();
    }

    public void stopShooter()
    {
        shooterMotor.stop();
    }

    public void setShooterPower(double power)
    {
        // if manualOverride call motor.setPower else call motor.setPidPower
    }

    public double getShooterPower()
    {
        return 0.0;
    }

    //
    // Tilter motor methods.
    //

    public void setTilterAngle(String owner, double angle, TrcEvent completionEvent, double timeout)
    {
        tilterMotor.setPosition(
            owner, 0.0, angle, true, RobotParams.Shooter.tilterPowerLimit, completionEvent, timeout);
    }

    public void setTilterAngle(String owner, double angle)
    {
        tilterMotor.setPosition(
            owner, 0.0, angle, true, RobotParams.Shooter.tilterPowerLimit, null, 0.0);
    }

    public void setTilterAngle(double angle)
    {
        tilterMotor.setPosition(
            null, 0.0, angle, true, RobotParams.Shooter.tilterPowerLimit, null, 0.0);
    }

    public double getTilterAngle()
    {
        return tilterMotor.getPosition();
    }

    public void setTilterPower(double power)
    {
        // if manualOverride call motor.setPower else call motor.setPidPower
    }

    public double getTilterPower()
    {
        return 0.0;
    }

}
