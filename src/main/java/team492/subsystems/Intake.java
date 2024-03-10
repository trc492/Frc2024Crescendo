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

import TrcCommonLib.trclib.TrcIntake;
import TrcCommonLib.trclib.TrcTriggerDigitalInput;
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcCANSparkMax;
import TrcFrcLib.frclib.FrcDigitalInput;
import team492.Robot;
import team492.RobotParams;

public class Intake
{
    private static final String moduleName = Intake.class.getSimpleName();

    private final Robot robot;
    public final FrcCANSparkMax intakeMotor;
    private final FrcDigitalInput entrySensor, exitSensor;
    private final TrcTriggerDigitalInput entryTrigger, exitTrigger;
    public final TrcIntake intake;

    public Intake(Robot robot)
    {
        this.robot = robot;
        intakeMotor = new FrcCANSparkMax(moduleName + ".motor", RobotParams.Intake.motorCandId, true);
        intakeMotor.resetFactoryDefault();
        intakeMotor.setMotorInverted(RobotParams.Intake.motorInverted);
        intakeMotor.setBrakeModeEnabled(true);
        intakeMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);

        entrySensor = new FrcDigitalInput(moduleName + ".entrySensor", RobotParams.Intake.entrySensorChannel);
        entrySensor.setInverted(RobotParams.Intake.entrySensorInverted);
        entryTrigger = new TrcTriggerDigitalInput(moduleName + ".entryTrigger", entrySensor);

        exitSensor = new FrcDigitalInput(moduleName + ".exitSensor", RobotParams.Intake.exitSensorChannel);
        exitSensor.setInverted(RobotParams.Intake.exitSensorInverted);
        exitTrigger = new TrcTriggerDigitalInput(moduleName + ".exitTrigger", exitSensor);

        intake = new TrcIntake(
            moduleName, intakeMotor, new TrcIntake.Trigger(entryTrigger, this::checkNote),
            new TrcIntake.Trigger(exitTrigger, this::checkNote));
    }   //Intake

    /**
     * This method returns the instance name.
     */
    @Override
    public String toString()
    {
        return intake.toString();
    }   //toString

    /**
     * This method returns the TrcIntake object created.
     *
     * @return TrcIntake object.
     */
    public TrcIntake getIntake()
    {
        return intake;
    }   //getIntake

    /**
     * This method is called when a trigger occurred to check if we got a Note.
     *
     * @param context not used.
     */
    private void checkNote(Object context)
    {
        if (robot.ledIndicator != null)
        {
            robot.ledIndicator.setIntakeDetectedObject(intake.hasObject());
        }
    }   //checkNote

}   //class Intake
