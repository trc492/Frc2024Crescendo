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

import frclib.motor.FrcCANSparkMax;
import team492.RobotParams;
import trclib.dataprocessor.TrcUtil;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;

public class Climber
{
    private static final String moduleName = Climber.class.getSimpleName();

    public final FrcCANSparkMax climberMotor;
    private boolean climbing = false;
    private boolean manualOverride = false;

    public Climber()
    {
        climberMotor = new FrcCANSparkMax(moduleName + ".motor", RobotParams.Climber.motorCandId, true);
        climberMotor.resetFactoryDefault();
        climberMotor.setMotorInverted(RobotParams.Climber.motorInverted);
        climberMotor.setBrakeModeEnabled(true);
        climberMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        climberMotor.enableLowerLimitSwitch(true);
        climberMotor.enableUpperLimitSwitch(true);
        climberMotor.setPositionSensorScaleAndOffset(RobotParams.Climber.posScale, 0.0);
        // We are using software position PID control for Climber. So we just enable software PID before setting
        // PID coefficients.
        climberMotor.setSoftwarePidEnabled(true);
        climberMotor.setPositionPidParameters(RobotParams.Climber.posPidCoeff, RobotParams.Climber.posPidTolerance);
        climberMotor.setPositionPidPowerComp(this::getClimbPowerComp);
    }   //Climber

    /**
     * This method returns the instance name.
     */
    @Override
    public String toString()
    {
        return moduleName;
    }   //toString

    public void zeroCalibrate()
    {
        climberMotor.motor.clearFaults();
        climberMotor.zeroCalibrate(RobotParams.Climber.calPower);
    }

    private double getClimbPowerComp(double power)
    {
        TrcDbgTrace.globalTraceDebug(moduleName, "Climbing...");
        return climbing? RobotParams.Climber.climbPowerComp: 0.0;
    }

    public double getPosition()
    {
        return climberMotor.getPosition();
    }
    
    public void extend(TrcEvent completionEvent, double timeout)
    {
        climbing = false;
        climberMotor.setPosition(0.0, RobotParams.Climber.maxHeight, true, 1.0, completionEvent, timeout);
    }

    public void extend()
    {
        climbing = false;
        climberMotor.setPosition(RobotParams.Climber.maxHeight);
    }

    public void retract(TrcEvent completionEvent, double timeout)
    {
        climbing = false;
        climberMotor.setPosition(0.0, RobotParams.Climber.minHeight, true, 1.0, completionEvent, timeout);
    }

    public void retract()
    {
        climbing = false;
        climberMotor.setPosition(RobotParams.Climber.minHeight);
    }

    public void climb()
    {
        climbing = true;
        climberMotor.setPosition(RobotParams.Climber.minHeight);
    }

    /**
     * This method enables/disables manual override for setClimberPower. When manual override is not enabled,
     * setClimberPower will operate by PID control which means it will slow down the approach when it's near the
     * upper or lower angle limits. When manul override is enabled, it will simply applied the specified power as is.
     *
     * @param enabled specifies true to enable manual override, false to disable.
     */
    public void setManualOverrideEnabled(boolean enabled)
    {
        manualOverride = enabled;
    }   //setManualOverrideEnabled

    /**
     * This method moves climber up and down with the specified power. It is typically used by TeleOp to control
     * the climber by a joystick value. Climber movement is PID controlled when manual override is not enabled.
     *
     * @param power specifies the power duty cycle used to move climber (in the range of -1 to 1).
     */
    public void setClimbPower(double power)
    {
        if (manualOverride)
        {
            climberMotor.setPower(null, 0.0, power, 0.0, null);;
        }
        else
        {
            climberMotor.setPidPower(null, power, RobotParams.Climber.minHeight, RobotParams.Climber.maxHeight, true);
        }
    }   //setClimberPower

}   //class Climber
