/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
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

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.settings.Constants.PHOTONVISION;

public class PhotonCameraWrapper {
    private static PhotonCameraWrapper instance;
    private PhotonCamera leftCamera;
    private PhotonCamera rightCamera;
    private PhotonPoseEstimator leftPhotonPoseEstimator;
    private PhotonPoseEstimator rightPhotonPoseEstimator;

    private PhotonCameraWrapper() {
        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        leftCamera = new PhotonCamera(PHOTONVISION.leftCameraName);
        rightCamera = new PhotonCamera(PHOTONVISION.rightCameraName);

        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are
            // on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            leftPhotonPoseEstimator = new PhotonPoseEstimator(
                    fieldLayout, PoseStrategy.MULTI_TAG_PNP, leftCamera, PHOTONVISION.robotToLeftCam);
            leftPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            rightPhotonPoseEstimator = new PhotonPoseEstimator(
                    fieldLayout, PoseStrategy.MULTI_TAG_PNP, rightCamera, PHOTONVISION.robotToLeftCam);
            rightPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if
            // we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            leftPhotonPoseEstimator = null;
            rightPhotonPoseEstimator = null;
        }
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and
     *         targets used to create
     *         the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFromLeftCam(Pose2d prevEstimatedRobotPose) {
        if (leftPhotonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        leftPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        return leftPhotonPoseEstimator.update();
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and
     *         targets used to create
     *         the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFromRightCam(Pose2d prevEstimatedRobotPose) {
        if (rightPhotonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        rightPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        return rightPhotonPoseEstimator.update();
    }

    public static PhotonCameraWrapper getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new PhotonCameraWrapper();
        }
        return instance;
    }
}