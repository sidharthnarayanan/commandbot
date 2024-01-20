package frc.robot.subsystems;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTag {
    public static void apriltagVisionThreadProc() {
        var detector = new AprilTagDetector();
        // look for tag36h11, correct 3 error bits
        detector.addFamily("tag36h11", 3);

        // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
        // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
        var poseEstConfig = new AprilTagPoseEstimator.Config(
                0.1651, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
        var estimator = new AprilTagPoseEstimator(poseEstConfig);

        // Get the UsbCamera from CameraServer
        UsbCamera camera = CameraServer.startAutomaticCapture();
        // Set the resolution
        camera.setResolution(640, 480);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Detected", 640, 480);

        // Mats are very memory expensive. Lets reuse these.
        var mat = new Mat();
        var grayMat = new Mat();

        // Instantiate once
        ArrayList<Long> tags = new ArrayList<>();
        var outlineColor = new Scalar(0, 255, 0);
        var crossColor = new Scalar(0, 0, 255);

        while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
            }

            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

            AprilTagDetection[] detections = detector.detect(grayMat);

            // have not seen any tags yet
            tags.clear();

            for (AprilTagDetection detection : detections) {
                // remember we saw this tag
                System.out.println("Tag family:"+detection.getFamily()+", ID:"+detection.getId());
                tags.add((long) detection.getId());

                // determine pose
                Transform3d pose = estimator.estimate(detection);

                // put pose into dashboard
                Rotation3d rot = pose.getRotation();
               System.out.println("Pos:"+pose.getX()+","+pose.getY()+","+pose.getZ()+","+rot.getX()+","+ rot.getY()+","+ rot.getZ());
            }

            // Give the output stream a new image to display
            outputStream.putFrame(mat);
        }

        detector.close();
    }
}
