/*
 * MIT License
 *
 * Copyright (c) PhotonVision
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

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.PhotonPoseEstimator.PoseStrategy;

import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

/** An estimated pose based on pipeline result */
public class EstimatedRobotPose {
    /** The best estimated pose */
    public final Pose3d bestEstimatedPose;

    /** The alternate estimated pose */
    public final Pose3d alternateEstimatedPose;

     /** Ambiguity of the best pose */
    public final double bestAmbiguity;

    /** Ambiguity of the alternate pose */
    public final double altAmbiguity;

    /** The estimated time the frame used to derive the robot pose was taken */
    public final double timestampSeconds;

    /** A list of the targets used to compute this pose */
    public final List<PhotonTrackedTarget> targetsUsed;

    /** The strategy actually used to produce this pose */
    public final PoseStrategy strategy;

    public final int[] fiducialIDsUsed;
    
    public final double[] ambiguities;

    public final double totalArea;

    /**
     * Constructs an EstimatedRobotPose
     *
     * @param estimatedPose estimated pose
     * @param timestampSeconds timestamp of the estimate
     */
    public EstimatedRobotPose(
            Pose3d bestEstimatedPose,
            Pose3d alternateEstimatedPose,
            double bestAmbiguity,
            double altAmbiguity,
            double timestampSeconds,
            List<PhotonTrackedTarget> targetsUsed,
            PoseStrategy strategy) {
                var targetsSeen = targetsUsed.size();
                var visibleFiducialIDs = new int[targetsSeen];
                var ambiguities = new double[targetsSeen];
        
                double area = 0;
                
                for (int i = 0; i < targetsSeen; i++) {
                    var target = targetsUsed.get(i);
                    visibleFiducialIDs[i] = target.getFiducialId();
                    ambiguities[i] = target.getPoseAmbiguity();
                    area += target.getArea() / 100; // Area is returned in percent but we want fraction
                    // See
                    // https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html#getting-data-from-a-target
                }
        this.bestEstimatedPose = bestEstimatedPose;
        this.alternateEstimatedPose = alternateEstimatedPose;
        this.bestAmbiguity = bestAmbiguity;
        this.altAmbiguity = altAmbiguity;
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = targetsUsed;
        this.strategy = strategy;
        this.ambiguities = ambiguities;
        this.fiducialIDsUsed = visibleFiducialIDs;
        this.totalArea = area;
    }
}
