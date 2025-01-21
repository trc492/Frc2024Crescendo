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

import frclib.motor.FrcServo;
import team492.RobotParams;
import trclib.timer.TrcTimer;

public class Deflector
{
    private static final String moduleName = Deflector.class.getSimpleName();

    private final FrcServo servo;
    private final TrcTimer timer;
    private boolean extended = false;

    public Deflector()
    {
        servo = new FrcServo(moduleName + ".servo", RobotParams.HWConfig.PWM_CHANNEL_DEFLECTOR);
        servo.setInverted(RobotParams.Deflector.inverted);
        timer = new TrcTimer(moduleName);
        retract();
    }   //Deflector

    /**
     * This method returns the instance name.
     */
    @Override
    public String toString()
    {
        return moduleName;
    }   //toString

    public void extend()
    {
        servo.setPosition(RobotParams.Deflector.extendPos);
        extended = true;
    }   //extend

    public void extend(double duration)
    {
        extend();
        timer.set(duration, this::extendTimeout);
    }   //extend

    private void extendTimeout(Object context)
    {
        retract();
    }   //extendTimeout

    public void retract()
    {
        servo.setPosition(RobotParams.Deflector.retractPos);
        extended = false;
    }   //retract

    public boolean isExtended()
    {
        return extended;
    }   //isExtended

}   //class Deflector
