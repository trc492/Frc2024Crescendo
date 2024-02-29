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
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcCANSparkMax;
import team492.RobotParams;

public class Climber
{
    private static final String moduleName = Climber.class.getSimpleName();

    public final FrcCANSparkMax climberMotor;
    private boolean climbing = false;

    public Climber()
    {
        climberMotor = new FrcCANSparkMax(moduleName + ".motor", RobotParams.Climber.motorCandId, true);
        climberMotor.resetFactoryDefault();
        climberMotor.setMotorInverted(RobotParams.Climber.motorInverted);
        climberMotor.setBrakeModeEnabled(true);
        climberMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        climberMotor.setPositionSensorScaleAndOffset(RobotParams.Climber.posScale, 0.0);
        climberMotor.setPositionPidCoefficients(RobotParams.Climber.posPidCoeff);
        climberMotor.enableLowerLimitSwitch(true);
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

}   //class Climber
