package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathConstants;

public class TrajectoryFactory {
    /**
     * Determines type of an inputted trajectory - constructed using Choreo, or using PathPlanner. Default should be Choreo.
     */
    public enum PathType {
        CHOREO,
        PATHPLANNER
    }

    /**Constructs a path following command to generate a path to a target position.
     * 
     * @param targetPose The desired end pose of the generated path.
     * @return The constructed path following command
     */
    public Command getPathFindToPoseCommand(Pose2d targetPose) {        

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            PathConstants.CONSTRAINTS,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        return pathfindingCommand;
    }

    /**Constructs a path following command to a preset path from the deploy directory Path can be PathPlanner or Choreo-constructed.
     * 
     * @param pathname A String containing the name of the file with the path (leave out the .traj or .path ending).
     * @param pathType A {@link PathType} determining the format of the inputted trajectory. Files ending in .path should be imported as PATHPLANNER, while files ending in .traj should be imported as CHOREO.
     * @return The constructed path following command
     */
    public Command getPathFindToPathCommand(String pathname, PathType pathType) {
        PathPlannerPath path;
        switch(pathType) {
            case CHOREO: 
                path = PathPlannerPath.fromChoreoTrajectory(pathname);
            case PATHPLANNER:
                path = PathPlannerPath.fromPathFile(pathname);
            default:
                path = PathPlannerPath.fromChoreoTrajectory(pathname);
        }
        
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
            path,
            PathConstants.CONSTRAINTS,
            3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        return pathfindingCommand;
    }

    /**
     * Constructs a path following command through a provided set of waypoints. Ends with desired holonomic rotation.
     * @param goalEndRotationHolonomic Desired holonomic end rotation
     * @param poses List of bezier poses. Each {@link Pose2d} represents one waypoint. The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
     * @return The constructed path following command through provided poses, with set end rotation.
     */
    public Command getPathFromWaypoints(Rotation2d goalEndRotationHolonomic, Pose2d...poses) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints, PathConstants.CONSTRAINTS,
            new GoalEndState(0.0, goalEndRotationHolonomic) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        Command pathCommand = AutoBuilder.pathfindThenFollowPath(
            path,
            PathConstants.CONSTRAINTS,
            3.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        return pathCommand;
    }
}
