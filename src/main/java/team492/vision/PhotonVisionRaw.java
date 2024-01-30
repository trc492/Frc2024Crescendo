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

import java.io.UncheckedIOException;
import java.util.Optional;


import TrcCommonLib.trclib.TrcTimer;
import TrcFrcLib.frclib.FrcPhotonVisionRaw;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import team492.subsystems.LEDIndicator;
import team492.vision.PhotonVision.PipelineType;

/**
 * This class is a thin wrapper extending FrcPhotonVision that provides additional game specific functionalities.
 */
public class PhotonVisionRaw extends FrcPhotonVisionRaw
{
    private final LEDIndicator ledIndicator;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    // private final PhotonPoseEstimator poseEstimator;
    private PipelineType currPipeline;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the network table name that PhotonVision is broadcasting information over.
     * @param ledIndicator specifies the LEDIndicator object, can be null if none provided.
     */
    public PhotonVisionRaw(String cameraName, LEDIndicator ledIndicator)
    {
        super(cameraName);
        this.ledIndicator = ledIndicator;

        double startTime = TrcTimer.getModeElapsedTime();
        try
        {
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
            // poseEstimator = new PhotonPoseEstimator(
            //     aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this, RobotParams.CAMERA_TRANSFORM3D);
            // poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
        catch (UncheckedIOException e)
        {
            throw new RuntimeException("Failed to load AprilTag field layout info.");
        }
        double endTime = TrcTimer.getModeElapsedTime();

        tracer.traceDebug(instanceName, "Loading AprilTag field layout took " + (endTime - startTime) + " sec.");
        setPipeline(PipelineType.APRILTAG);
    }   //PhotonVisionRaw

    /**
     * This method returns the best detected object and set the LED to indicate type detected object type.
     *
     * @return best detected object.
     */
    @Override
    public DetectedObject getDetectedObject()
    {
        DetectedObject detectedObject = super.getDetectedObject();

        if (detectedObject != null && ledIndicator != null)
        {
            ledIndicator.setPhotonDetectedObject(getPipeline());
        }

        return detectedObject;
    }   //getDetectedObject

    /**
     * This method returns the 3D field location of the AprilTag with its given ID.
     *
     * @param aprilTagId sepcifies the AprilTag ID to retrieve its field location.
     * @return 3D location of the AprilTag.
     */
    public Pose3d getAprilTagPose(int aprilTagId)
    {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(aprilTagId);
        return tagPose.isPresent()? tagPose.get(): null;
    }   //getAprilTagPose

    // /**
    //  * This method returns the absolute field location of the camera with the given detected AprilTag object.
    //  *
    //  * @param detectedObj specifies the AprilTag object detected by the camera.
    //  * @return camera's absolute field location.
    //  */
    // public TrcPose2D getRobotFieldPosition(DetectedObject detectedObj)
    // {
    //     TrcPose2D robotPose = null;
    //     int aprilTagId = (Integer) detectedObj.target;
    //     // aprilTagPose is the absolute field position of the AprilTag.
    //     Pose3d aprilTagPose = getAprilTagPose(aprilTagId);

    //     if (aprilTagPose != null)
    //     {
    //         // camPose3d is the absolute field position of the camera.
    //         Pose3d camPose3d = aprilTagPose.transformBy(detectedObj.target.getBestCameraToTarget().inverse());
    //         // robotPose3d is the absolute 3D field position of the robot centroid on the ground.
    //         Pose3d robotPose3d = camPose3d.transformBy(RobotParams.CAMERA_TRANSFORM3D.inverse());
    //         // robotPose is the absolute field position of the robot adjusted to the robot coordinate system.
    //         robotPose = DetectedObject.pose3dToTrcPose2D(robotPose3d);
    //         tracer.traceDebug(
    //             instanceName,
    //             "[" + aprilTagId + "] camPose3d=" + camPose3d +
    //             ", robotPose3d=" + robotPose3d +
    //             ", RobotPose=" + robotPose);
    //     }

    //     return robotPose;
    // }   //getRobotFieldPosition

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
            super.selectPipeline(pipelineType.pipelineIndex);
        }
    }   //setPipeline

    /**
     * This method returns the active pipeline of the LimeLight.
     *
     * @return active pipeline.
     */
    public PipelineType getPipeline()
    {
        currPipeline = PipelineType.getType(super.getSelectedPipeline());
        return currPipeline;
    }   //getPipeline

}   //class PhotonVisionRaw
