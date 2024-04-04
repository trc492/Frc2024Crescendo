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

package team492.subsystems;

import TrcCommonLib.trclib.TrcAddressableLED;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcDriveBase.DriveOrientation;
import TrcFrcLib.frclib.FrcAddressableLED;
import TrcFrcLib.frclib.FrcColor;
import team492.RobotParams;
import team492.vision.PhotonVision;

public class LEDIndicator
{
    private static final TrcAddressableLED.Pattern aprilTagLockedPattern =  // Magenta
        new TrcAddressableLED.Pattern("AprilTagLocked", new FrcColor(63, 0, 63), RobotParams.HWConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern aprilTagPattern =        // Green
        new TrcAddressableLED.Pattern("AprilTag", new FrcColor(0, 63, 0), RobotParams.HWConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern intakeActivePattern =    // Yellow
        new TrcAddressableLED.Pattern("IntakeActive", new FrcColor(63, 63, 0), RobotParams.HWConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern notePattern =            // Orange
        new TrcAddressableLED.Pattern("Note", new FrcColor(80, 15, 0), RobotParams.HWConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern seeNothingPattern =      // Red
        new TrcAddressableLED.Pattern("SeeNothing", new FrcColor(63, 0, 0), RobotParams.HWConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern intakeHasNotePattern =   // Cyan
        new TrcAddressableLED.Pattern("GotNote", new FrcColor(0, 63, 63), RobotParams.HWConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern fieldOrientedPattern =   // White
        new TrcAddressableLED.Pattern("FieldOriented", new FrcColor(63, 63, 63), RobotParams.HWConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern robotOrientedPattern =   // Blue
        new TrcAddressableLED.Pattern("RobotOriented", new FrcColor(0, 0, 63), RobotParams.HWConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern inverseOrientedPattern = // Magenta
        new TrcAddressableLED.Pattern("InverseOriented", new FrcColor(63, 0, 63), RobotParams.HWConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern nominalPattern =         // Black
        new TrcAddressableLED.Pattern("Nominal", new FrcColor(0, 0, 0), RobotParams.HWConfig.NUM_LEDS);

    private static final TrcAddressableLED.Pattern[] priorities =
        new TrcAddressableLED.Pattern[]
        {
            // Highest priority.
            aprilTagLockedPattern,
            aprilTagPattern,
            intakeActivePattern,
            intakeHasNotePattern,
            notePattern,
            seeNothingPattern,
            fieldOrientedPattern,
            robotOrientedPattern,
            inverseOrientedPattern,
            nominalPattern
            // Lowest priority
        };

    private FrcAddressableLED led;
    private Boolean shooterVelOnTarget = null;

    /**
     * Constructor: Create an instance of the object.
     */
    public LEDIndicator()
    {
        led = new FrcAddressableLED("LED", RobotParams.HWConfig.NUM_LEDS, RobotParams.HWConfig.PWM_CHANNEL_LED);
        reset();
    }   //LEDIndicator

    /**
     * This method resets the LED strip to the nominal pattern.
     */
    public void reset()
    {
        led.setEnabled(true);
        led.setPatternPriorities(priorities);
        led.reset();
        led.resetAllPatternStates();
        led.setPatternState(nominalPattern, true);
    }   //reset

    /**
     * This method sets the LED to indicate the drive orientation mode of the robot.
     *
     * @param orientation specifies the drive orientation mode.
     */
    public void setDriveOrientation(DriveOrientation orientation)
    {
        switch (orientation)
        {
            case INVERTED:
                led.setPatternState(inverseOrientedPattern, true);
                led.setPatternState(robotOrientedPattern, false);
                led.setPatternState(fieldOrientedPattern, false);
                break;

            case ROBOT:
                led.setPatternState(inverseOrientedPattern, false);
                led.setPatternState(robotOrientedPattern, true);
                led.setPatternState(fieldOrientedPattern, false);
                break;

            case FIELD:
                led.setPatternState(inverseOrientedPattern, false);
                led.setPatternState(robotOrientedPattern, false);
                led.setPatternState(fieldOrientedPattern, true);
                break;
        }
    }   //setDriveOrientation

    public void setShooterVelOnTarget(Boolean onTarget)
    {
        this.shooterVelOnTarget = onTarget;
    }   //setShooterVelOnTarget

    /**
     * This method sets the LED to indicate the type of Photon Vision detected object.
     *
     * @param pipelineType specifies the detected object type (by its pipeline), null if none detected.
     * @param objPose specifies the detected object pose, valid if pipelineType is not null.
     */
    public void setPhotonDetectedObject(PhotonVision.PipelineType pipelineType, TrcPose2D objPose)
    {
        if (pipelineType == null)
        {
            led.setPatternState(seeNothingPattern, true, 0.5);
        }
        else
        {
            switch (pipelineType)
            {
                case APRILTAG:
                    // led.setPatternState(aprilTagPattern, true, 0.5);
                    if (Math.abs(objPose.angle) < RobotParams.Vision.ONTARGET_THRESHOLD &&
                        (shooterVelOnTarget == null || shooterVelOnTarget))
                    {
                        led.setPatternState(aprilTagLockedPattern, true, 0.5);
                    }
                    else
                    {
                        led.setPatternState(aprilTagPattern, true, 0.5);
                    }
                    break;

                case NOTE:
                    led. setPatternState(notePattern, true, 0.5);
                    // if (Math.abs(objPose.angle) < RobotParams.Vision.ONTARGET_THRESHOLD)
                    // {
                    //     led.setPatternState(notePattern, true, 0.1, 0.1);
                    // }
                    // else
                    // {
                    //     led.setPatternState(notePattern, true, 0.5);
                    // }
                    break;
            }
        }
    }   //setPhotonDetectedObject

    /**
     * This method sets the LED to indicate Intake detected object.
     *
     * @param hasObject specifies true if Intake has an object, false otherwise.
     */
    public void setIntakeDetectedObject(boolean hasObject)
    {
        if (hasObject)
        {
            led.setPatternState(intakeHasNotePattern, true, 0.25, 0.25);
        }
        else
        {
            led.setPatternState(intakeHasNotePattern, false);
        }
    }   //setIntakeDetectedObject

    /**
     * This method sets the LED to indicate Intake is in active state.
     *
     * @param hasObject specifies true if Intake is active, false otherwise.
     */
    public void setIntakeActive(boolean active)
    {
        if (active)
        {
            led.setPatternState(intakeActivePattern, true, 0.25, 0.25);
        }
        else
        {
            led.setPatternState(intakeActivePattern, false);
        }
    }   //setIntakeActive

}   //class LEDIndicator
