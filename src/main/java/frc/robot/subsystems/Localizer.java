// LOCALIZER: accesses drivetrain for odometry and AprilTagFinder for vision measurements

package frc.robot.subsystems;
import java.lang.reflect.Member;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AprilTagFinder.VisionMeasurement;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Localizer extends SubsystemBase
{
    private Drivetrain driveTrain;
    private SwerveDrivePoseEstimator estimator;
    private FieldMap fieldMap;
    private AprilTagFinder finder;
    private double lastUpdateTime;
    private SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] swerveModulePositions;
    private Matrix<N3, N1> measurementStdDev = VecBuilder.fill(0.5, 0.5, 0.5); //this actual creates the 3 by 1 matrix
    private int measurementCounter = 0;
    private double StdDevX = 0.5;
    private double StdDevY = 0.5;
    private double StdDevA = 0.5;
    private double timeGap = 1.0;
    private double linearSpeedThreshold = 2.5; // TODO: get actual numbers here
    private double angularSpeedThreshold = 1.5; // TODO: get actual numbers here

    //added a set transform from sensor to center of the robot to the sensor and can have multiple as needed
    private final Transform3d sensorTransform = new Transform3d();
    public Localizer(Drivetrain driveTrain, FieldMap fieldMap, AprilTagFinder finder)
    {
        this.driveTrain = driveTrain;
        this.fieldMap = fieldMap;
        this.finder = finder;
        this.kinematics = driveTrain.getKinematics();
        this.swerveModulePositions = driveTrain.getSwerveModulePositions();

        estimator = new SwerveDrivePoseEstimator(
            kinematics, driveTrain.getOdometry().getRotation(), swerveModulePositions, new Pose2d()
        );
        lastUpdateTime = Timer.getFPGATimestamp();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Localizer");
        builder.addDoubleProperty("StdDev X", this::getStdDevX, this::setStdDevX);
        builder.addDoubleProperty("StdDev Y", this::getStdDevY, this::setStdDevY);
        builder.addDoubleProperty("StdDev Angle", this::getStdDevA, this::setStdDevA);
        builder.addDoubleProperty("Time between updates", this::getTime, this::setTime);
        builder.addDoubleProperty("Linear Speed Thres", this::getLinearSpeed, this::setLinearSpeed);
        builder.addDoubleProperty("Angular Speed Thres", this::getAngularSpeed, this::setAngularSpeed);
    }

    public double getTime() {
        return timeGap;
    }

    public void setTime(double time) {
        timeGap = time;
    }
    
    public double getStdDevX() {
        return StdDevX;
    }

    public void setStdDevX(double newX) {
        StdDevX = newX;
    }

    public double getStdDevY() {
        return StdDevY;
    }

    public void setStdDevY(double newY) {
        StdDevX = newY;
    }

    public double getStdDevA() {
        return StdDevA;
    }

    public void setStdDevA(double newA) {
        StdDevX = newA;
    }

    public void resetPos(Pose2d newPos) {
        estimator.resetPose(newPos);
    }

    public double getLinearSpeed() {
        return linearSpeedThreshold;
    }

    public void setLinearSpeed(double speed) {
        linearSpeedThreshold = speed;
    }

    public double getAngularSpeed() {
        return angularSpeedThreshold;
    }

    public void setAngularSpeed(double angularSpeed) {
        angularSpeedThreshold = angularSpeed;
    }

    
    
    @Override
    public void periodic()
    {
        double now = Timer.getFPGATimestamp();
        estimator.updateWithTime(now, driveTrain.getOdometry().getRotation(), swerveModulePositions);
        /*
         * if (have a new sensor measurement && it is valid)
         * {
         *  transform sensor measurement to a measurement of where the robot is on the map
         *  apply update to estimator
         * }
         */
        // only run sensor update if we've moved enough and a few seconds have passed
        if (now - lastUpdateTime > timeGap && measurementStable())
        {
            ArrayList<AprilTagFinder.VisionMeasurement> measurements = finder.getMeasurements();

            for(int index = 0; index < measurements.size(); index++) 
            {
                VisionMeasurement currentMeasurement = measurements.get(index);
                //TODO: compute terms based on range to target  
                updateStdDevs(currentMeasurement);

                estimator.addVisionMeasurement(currentMeasurement.pose, currentMeasurement.timeStamp, measurementStdDev);
                measurementCounter++;
            }
            lastUpdateTime = now;
            SmartDashboard.putNumber("Localize Measurements", measurementCounter);
        }
    }

    public Pose2d getPose()
    {
        return estimator.getEstimatedPosition();
    }

    public void additionalSensorMeasurement(int id, FieldMap fieldMap)
    {
        // TODO:
        // null needs to be the sensor input on the line below
        Transform3d transform3d = new Transform3d(new Pose3d(), null); 
        Pose2d landmarkPose;
                //transforms the position of the landmark by the transform 
        
        
        // Pose3d measurement = landmarkPose.transformBy(transform3d); 
        // measurement = measurement.transformBy(sensorTransform);
        // Pose2d measurement2d = new Pose2d(
        //     new Translation2d(measurement.getX(), measurement.getY()),
        //     new Rotation2d(measurement.getRotation().getAngle()) 
        // );
        // estimator.addVisionMeasurement(measurement2d, Timer.getFPGATimestamp());
    }

    private boolean measurementStable()
    {
        double linearSpeed = Math.sqrt(Math.pow(driveTrain.getChassisSpeeds().vxMetersPerSecond, 2) + 
                                    Math.pow(driveTrain.getChassisSpeeds().vxMetersPerSecond, 2));
        double angularSpeed = Math.abs(driveTrain.getChassisSpeeds().omegaRadiansPerSecond);
        if (linearSpeed <= linearSpeedThreshold && angularSpeed <= angularSpeedThreshold)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    private void updateStdDevs(AprilTagFinder.VisionMeasurement measurement)
    {
        measurementStdDev.set(0, 0, StdDevX + 0.1 * measurement.range); //x standard deviation
        measurementStdDev.set(1, 0, StdDevY + 0.1 * measurement.range); //y standard deviation
        measurementStdDev.set(2, 0, StdDevA + 0.1 * measurement.range); //angle standard deviation
    }
}