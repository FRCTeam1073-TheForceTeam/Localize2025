package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class PhotonVisionConstants {
        public static class front_left_cam {
            public static final String kCameraName = "front_left";
            public static final Transform3d kRobotToCam = new Transform3d(
                    new Translation3d(Units.inchesToMeters(9.812), Units.inchesToMeters(9.29),   //CHANGE #
                            Units.inchesToMeters(8.693)),
                    new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(0)));
        }

        public static class front_right_cam {
            public static final String kCameraName = "front_right";
            public static final Transform3d kRobotToCam = new Transform3d(
                    new Translation3d(.159, -.213,.53),
                    new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(0)));
        }
       

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);   //CHANGE #
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    }
}