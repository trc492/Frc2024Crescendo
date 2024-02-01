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

package team492.vision;

import TrcFrcLib.frclib.FrcPhotonVision;
import team492.subsystems.LEDIndicator;

/**
 * This class is a thin wrapper extending FrcPhotonVision that provides additional game specific functionalities.
 */
public class PhotonVision extends FrcPhotonVision
{
    public enum PipelineType
    {
        APRILTAG(0),
        NOTE(1);

        public int pipelineIndex;

        PipelineType(int value)
        {
            pipelineIndex = value;
        }

        public static PipelineType getType(int index)
        {
            PipelineType type = null;

            for (PipelineType pipelineType: PipelineType.values())
            {
                if (index == pipelineType.pipelineIndex)
                {
                    type = pipelineType;
                }
            }

            return type;
        }   //getType

    }   //enum PipelineType

    private final LEDIndicator ledIndicator;
    private PipelineType currPipeline = PipelineType.APRILTAG;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the network table name that PhotonVision is broadcasting information over.
     * @param ledIndicator specifies the LEDIndicator object, can be null if none provided.
     */
    public PhotonVision(String cameraName, LEDIndicator ledIndicator)
    {
        super(cameraName);
        this.ledIndicator = ledIndicator;
        setPipeline(currPipeline);
    }   //PhotonVision

    /**
     * This method returns the best detected object and set the LED to indicate type detected object type.
     *
     * @return best detected object.
     */
    @Override
    public DetectedObject getBestDetectedObject()
    {
        DetectedObject detectedObject = super.getBestDetectedObject();

        if (detectedObject != null && ledIndicator != null)
        {
            ledIndicator.setPhotonDetectedObject(getPipeline());
        }

        return detectedObject;
    }   //getBestDetectedObject

    /**
     * This method sets the active pipeline type used in the LimeLight.
     *
     * @param pipelineType specifies the pipeline to activate in the LimeLight.
     */
    public void setPipeline(PipelineType pipelineType)
    {
        if (pipelineType != currPipeline)
        {
            currPipeline = pipelineType;
            super.setPipelineIndex(pipelineType.pipelineIndex);
            // setLED(VisionLEDMode.kOff);
        }
    }   //setPipeline

    /**
     * This method returns the active pipeline of the LimeLight.
     *
     * @return active pipeline.
     */
    public PipelineType getPipeline()
    {
        currPipeline = PipelineType.getType(getPipelineIndex());
        return currPipeline;
    }   //getPipeline

}   //class PhotonVision
