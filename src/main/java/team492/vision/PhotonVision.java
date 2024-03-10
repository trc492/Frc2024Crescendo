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

import java.io.IOException;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcPose3D;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcPhotonVision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

    private final Transform3d robotToCam;
    private final TrcPose2D robotToCamPose;
    private final LEDIndicator ledIndicator;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private PipelineType currPipeline = PipelineType.APRILTAG;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the network table name that PhotonVision is broadcasting information over.
     * @param robotToCam specifies the 3D transform location of the camera from robot center.
     * @param robotToCamPose specifies the 2D camera pose from robot center.
     * @param ledIndicator specifies the LEDIndicator object, can be null if none provided.
     */
    public PhotonVision(String cameraName, Transform3d robotToCam, TrcPose2D robotToCamPose, LEDIndicator ledIndicator)
    {
        super(cameraName, robotToCam);
        this.robotToCam = robotToCam;
        this.robotToCamPose = robotToCamPose;
        this.ledIndicator = ledIndicator;

        double startTime = TrcTimer.getCurrentTime();
        try
        {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        }
        catch (IOException e)
        {
            throw new RuntimeException("Failed to load AprilTag field layout info.");
        }
        double endTime = TrcTimer.getCurrentTime();
        tracer.traceDebug(instanceName, "Loading AprilTag field layout took " + (endTime - startTime) + " sec.");

        setPipeline(currPipeline);
    }   //PhotonVision

    /**
     * This method returns the 3D field location of the AprilTag with its given ID.
     *
     * @param aprilTagId sepcifies the AprilTag ID to retrieve its field location.
     * @return 3D location of the AprilTag.
     */
    public Pose3d getAprilTagFieldPose3d(int aprilTagId)
    {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(aprilTagId);
        return tagPose.isPresent()? tagPose.get(): null;
    }   //getAprilTagFieldPose3d

    /**
     * This method returns the 3D field location of the AprilTag with its given ID.
     *
     * @param aprilTagId sepcifies the AprilTag ID to retrieve its field location.
     * @return 3D location of the AprilTag.
     */
    public TrcPose3D getAprilTagFieldPose(int aprilTagId)
    {
        Pose3d pose3d = getAprilTagFieldPose3d(aprilTagId);
        Rotation3d rotation = pose3d != null? pose3d.getRotation(): null;

        return pose3d == null? null:
                new TrcPose3D(-pose3d.getY() * TrcUtil.INCHES_PER_METER,
                              pose3d.getX() * TrcUtil.INCHES_PER_METER,
                              pose3d.getZ() * TrcUtil.INCHES_PER_METER,
                              -Math.toDegrees(rotation.getZ()),
                              -Math.toDegrees(rotation.getY()),
                              Math.toDegrees(rotation.getX()));
    }   //getAprilTagFieldPose

    /**
     * This method returns the robot's field position.
     *
     * @param aprilTagInfo specifies the detected AprilTag info.
     * @param usePoseEstimator specifies true to use PhotonVision Lib pose estimator, false to use the AprilTag field
     *        pose to calculate it ourselves.
     * @return robot's field location.
     */
    public TrcPose2D getRobotFieldPose(DetectedObject aprilTagInfo, boolean usePoseEstimator)
    {
        return usePoseEstimator? getRobotEstimatedPose(robotToCam):
                                 getRobotPoseFromAprilTagFieldPose(
                                    getAprilTagFieldPose(aprilTagInfo.target.getFiducialId()).toPose2D(),
                                    aprilTagInfo.targetPose,
                                    robotToCamPose);
    }   //getRobotFieldPose

    /**
     * This method returns the best detected object.
     *
     * @return best detected object.
     */
    @Override
    public DetectedObject getBestDetectedObject()
    {
        DetectedObject bestDetectedObj = super.getBestDetectedObject();

        if (bestDetectedObj != null && ledIndicator != null)
        {
            ledIndicator.setPhotonDetectedObject(getPipeline());
        }

        return bestDetectedObj;
    }   //getBestDetectedObject

    /**
     * This method finds a matching AprilTag ID in the specified array and returns the found index.
     *
     * @param id specifies the AprilTag ID to be matched.
     * @param aprilTagIds specifies the AprilTag ID array to find the given ID.
     * @return index in the array that matched the ID, -1 if not found.
     */
    private int matchAprilTagId(int id, int[] aprilTagIds)
    {
        int matchedIndex = -1;

        for (int i = 0; i < aprilTagIds.length; i++)
        {
            if (id == aprilTagIds[i])
            {
                matchedIndex = i;
                break;
            }
        }

        return matchedIndex;
    }   //matchAprilTagId

    /**
     * This method get the best detected AprilTag matching the specified AprilTag IDs array sorted by most preferred
     * ID at the top.
     *
     * @param aprilTagIds specifies the AprilTag IDs to look for.
     * @return best detected AprilTag.
     */
    public DetectedObject getBestDetectedAprilTag(int... aprilTagIds)
    {
        DetectedObject bestObj = null;

        if (currPipeline == PipelineType.APRILTAG)
        {
            DetectedObject objects[] = super.getDetectedObjects();

            if (objects != null)
            {
                int bestIdIndex = -1;
                for (DetectedObject obj: objects)
                {
                    int id = obj.target.getFiducialId();
                    int idIndex = matchAprilTagId(id, aprilTagIds);

                    if (idIndex != -1 && (bestIdIndex == -1 || idIndex < bestIdIndex))
                    {
                        // Found first match or a better match.
                        bestObj = obj;
                        bestIdIndex = idIndex;
                    }
                }
            }
        }

        if (bestObj != null && ledIndicator != null)
        {
            ledIndicator.setPhotonDetectedObject(currPipeline);
        }

        return bestObj;
    }   //getBestDetectedAprilTag

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

    //
    // Implements FrcPhotonVision abstract methods.
    //

    /**
     * This method returns the ground offset of the detected target.
     *
     * @return target ground offset.
     */
    public double getTargetGroundOffset(PhotonTrackedTarget target)
    {
        double targetHeight = 0.0;
        PipelineType pipelineType = getPipeline();

        switch (pipelineType)
        {
            case APRILTAG:
                if (target != null)
                {
                    // Even though PhotonVision said detected target, FieldLayout may not give us AprilTagPose.
                    // Check it before access the AprilTag pose.
                    Pose3d aprilTagPose = getAprilTagFieldPose3d(target.getFiducialId());
                    if (aprilTagPose != null)
                    {
                        targetHeight = aprilTagPose.getZ();
                    }
                }
                break;

            case NOTE:
                targetHeight = 0.0;
                break;
        }

        return targetHeight;
    }   //getTargetGroundOffset

}   //class PhotonVision
