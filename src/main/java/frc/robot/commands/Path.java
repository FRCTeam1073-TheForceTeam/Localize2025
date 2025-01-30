
package frc.robot.commands;
import java.util.ArrayList;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class represents a robot path wiht 2D points on a plane and desired headings in radians.
 * The path is a sequence of PathSegments. Paths are annotated with desired velocities and
 * other control information for each segment.
 * 
 * The path is defined and operates in field-centric orientation.
 */
public class Path 
{

    /**
     * The point is a point along the path. It has a 2D vector location and a blend_radius for 
     * defining if we are "close enough" to it.
     */
    public static class Point 
    {

        public Vector<N2> position = new Vector<N2>(N2.instance);
        public double     blend_radius = 0.2;     // Radius of bend/arrival at the point.

        public Point() {
            position.set(0,0,0.0);
            position.set(1,0,0.0);
        }

        public Point(double x, double y) {
            position.set(0,0,x);
            position.set(1,0,y);
        }

    }

    /**
     * A segment is a linear segment with a start and end point as well as a desired velocity
     * along the segment and a desired orientation along the segment.
     * 
     * The segment can also store commands to schedule at entry and exit of the segment
     * to coordinate actions for paths.
     * 
     * Note: The segment class precomputes the normalized direciton vector and length from start to
     * end positions when created. If you change start or end you need to call updateDirection()
     * to adjust the direction vector. If start and end are equal points, the direction
     * is the zero vector.
     */
    public static class Segment {
        public Point start;           // Start point on the segment. meters.
        public Point end;             // End point of the segment. meters.
        public Vector<N2> dir;        // Computed direction vector normal for segment.
        public double length;         // Length of the segment from start to end.
        public double orientation = 0.0;    // Desired orientation along the segment. Radians in field centric frame
        public double velocity = 1.0;       // Desired velocity along the segment. Shoudl always be positive m/s
        public double width = 1.0;    // Path width beyond which we stop trying to progress along the path.
        public double orientation_weight = 1.0; // Desired orientation weight along this segment.
        public double translation_weight = 1.0; // Desired translation weight along this segment.
        public Command entryCommand = null; // Command to schedule when segment is entered.
        public Command exitCommand = null;  // Command to schedule when segment is exited.
        public Activate entryActivate = null;
        public Activate exitActivate = null;
        public boolean entryActivateValue = false;
        public boolean exitActivateValue = false;

        public Segment(Point start, Point end, double orientation, double velocity) {
            this.start = start;
            this.end = end;
            updateDirection();
            this.orientation = orientation;
            this.velocity = velocity;
        }

        public void updateDirection() {
            length = end.position.minus(start.position).norm();

            // Compute a direction vector for the segment.
            if (length > 0.001) {
                this.dir = end.position.minus(start.position).div(length);
            } else {
                this.dir = new Vector<N2>(N2.instance);
                this.dir.set(0,0,0.0);
                this.dir.set(1,0,0.0);
            }
        }
    }

    public static class PathFeedback {
        Vector<N2> velocity;
        Pose2d pose;
        double orientation_weight = 1.0;
        double translation_weight = 1.0;

        public PathFeedback(Vector<N2> velocity, Pose2d pose, double ow, double tw) {
            this.velocity = velocity;
            this.pose = pose;
            this.orientation_weight = ow;
            this.translation_weight = tw;
        }
    }


    public ArrayList<Segment> segments; // The path is a list of segments.
    public double finalOrientation = 0.0;    // The orientation goal to hit at the end of the entire path.
    public double transverseVelocity = 1.0; // Maximum velocity for getting onto path (transverse velocity)
    // public double travelGain = 1.0; // Gain applied to traveling along path.

    /**
     * Create a path with a set of segments and a given final orientation where we stop.
     * @param segments
     * @param finalOrientation
     */
    public Path(ArrayList<Segment> segments, double finalOrientation) 
    {
        this.segments = segments;
        this.finalOrientation = finalOrientation;
    }

    /**
     * Return the position portion of a pose as a 2D Vector.
     * @param pose
     * @return
     */
    public static Vector<N2> positionToVector(Pose2d pose) {
        Vector<N2> v = new Vector<N2>(N2.instance);
        v.set(0,0, pose.getX());
        v.set(1,0, pose.getY());
        return v;
    }

    /**
     * Utility function returning distance to line segment defined by start and end from point p.
     * 
     * @param start
     * @param end
     * @param length
     * @param p
     * @param projection
     * @return
     */
    public double distanceToSegment(Vector<N2> start, Vector<N2> end, double length, Vector<N2> p, Vector<N2> projection) {
        if (length < 0.001)  
        {
            return (p.minus(start)).norm(); // Degenerate case
        } 
        else 
        {
            // Parametric line point:
            double t = p.minus(start).dot(end.minus(start)) / length;
            // Clamp to a point on the segment.
            if (t > 1.0) t = 1.0;
            if (t < 0.0) t = 0.0;
            var proj = start.plus(end.minus(start).times(t));
            // Store the projected point location if argument is not null.
            if (projection != null) 
            {
                projection.set(0,0,proj.get(0,0));
                projection.set(1,0, proj.get(1,0));
            }
            return p.minus(proj).norm();
        }
    }

    /**
     * Find the index of the segment that is closest to the given location. This is used
     * to start following a path by beginning at the closest segment.
     * 
     * If there are segments that are equidistant we pick the segment where the
     * endpoint is closest.
     * 
     * @param location
     * @return index of closest segment or -1 if something is wrong.
     */
    public int closestSegment(Pose2d location) {
        Vector<N2> pos = positionToVector(location); // Extract the position as a vector.

        // Iterate over all segments
        int closestIndex = -1;
        double closestDistance = 9999.9;

        for (int segmentIndex = 0; segmentIndex < segments.size(); ++segmentIndex) {
            Segment seg = segments.get(segmentIndex);

            double segDist = distanceToSegment(seg.start.position, seg.end.position, seg.length, pos, null);
            if (segDist < closestDistance) {
                closestIndex = segmentIndex;
                closestDistance = segDist;
            }
        }

        return closestIndex;
    }

    

    /**
     * Returns the path velocity for the given segment index and drive location.
     * 
     * This combines the closing velocity and the path velocity for a given location.
     * 
     * @param segmentIndex
     * @return
     */
    public PathFeedback getPathFeedback(int segmentIndex, Pose2d location) 
    {
        
        Vector<N2> Vp = new Vector<N2>(N2.instance);  // Output path velocity.
        Vector<N2> pos = positionToVector(location);  // Where are are.
        double proportion;

        Segment seg = segments.get(segmentIndex);
        
        // Scale the response based on how close we are to the path. If we are far
        // we don't go along path, if we are close we go along the path.
        Vector<N2> path_pos = new Vector<N2>(N2.instance);
        // Compute projection of current position onto path segment and the offset from the path segment.
        double path_offset = distanceToSegment(seg.start.position, seg.end.position, seg.length, pos, path_pos);
        SmartDashboard.putNumber("Path/path_offset", path_offset);

        if (path_offset < seg.width) 
        {
            proportion = path_offset / seg.width; /// 0 when we're dead-on, 1 when were at the offset.
            // Drive along the path and towards the path in proportion to error:
            Vp = (path_pos.minus(pos).times(proportion * transverseVelocity)).plus(seg.dir.times((1.0 - proportion) * seg.velocity));
            SmartDashboard.putNumber("Path/proportion", proportion);
        } 
        else
        {
            proportion = -1;
            SmartDashboard.putNumber("Path/proportion", proportion);
            // Drive only toward the path to get back on it.
            Vp = path_pos.minus(pos).div(path_offset).times(transverseVelocity);
        }

        // Set our leading position
        
        // Compute leading position by moving along path direction 0.5 second from closest.
        Vector<N2> pp = path_pos.plus(seg.dir.times(0.5*seg.velocity)); // 1/2 second ahead of projected point.
        pp.set(0,0,pp.get(0,0));
        pp.set(1,0,pp.get(1,0));   
        
        // Project the computed position onto the path segment as well to stay on segment and hit endpoints exactly.
        Vector<N2> ppp = new Vector<N2>(N2.instance);  /// projected path position point.
        double notused = distanceToSegment(seg.start.position, seg.end.position, seg.length, pp, ppp);

        Pose2d pathPoint = new Pose2d(new Translation2d(ppp.get(0, 0), ppp.get(1, 0)), new Rotation2d(seg.orientation));

        return new PathFeedback(Vp, pathPoint, seg.orientation_weight, seg.translation_weight);
    }

    /**
     * Return the desired orientation of the given path segment.
     * @param segmentIndex
     * @param location
     * @return
     */
    public double getPathOrientation(int segmentIndex, Pose2d location) {
        return segments.get(segmentIndex).orientation;
    }

    /**
     * Return true if the pose is at the endpoint of the given segmentIndex.
     * This means the location is within the blend_radius of the end point
     * of the given segment.
     * 
     * @param segmentIndex
     * @param location
     * @return
     */
    public boolean atEndPoint(int segmentIndex, Pose2d location) {
        Vector<N2> pos = positionToVector(location);
        double dist = segments.get(segmentIndex).end.position.minus(pos).norm();

        if (dist < segments.get(segmentIndex).end.blend_radius)
            return true;
        else
            return false;
    }


}