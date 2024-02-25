import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//sm imports sorry AHAHAH

public class Vision {
    private static final Scalar TARGET_COLOR = new Scalar(0, 255, 0);
    private static final Scalar TAG_COLOR = new Scalar(255, 0, 0);
    private static final double TAG_SIZE = 0.1; // size of tag, in meters (can change later)

    public static void main(String[] args) {
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("photonvision");

        // start cam
        VideoCapture camera = new VideoCapture();
        camera.open(0); // 0 is the default camera, change if needed
        camera.set(Videoio.CAP_PROP_FRAME_WIDTH, 640);
        camera.set(Videoio.CAP_PROP_FRAME_HEIGHT, 480);

        //THIS IS FOR TESTING NOT ACTUAL ROBOT USAGE - generic image processing (was messing around with it before apriltags usage)
        // Create a new thread for image processing
        new Thread(() -> {
            Mat frame = new Mat();
            while (true) {
                if (camera.read(frame)) {
                    // general image processing using something like OpenCV
                    Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2GRAY);

                    // update networktables with fresh data
                    table.getEntry("ProcessedFrame").setString("Data");

                    // release frame
                    frame.release();
                }
            }
        }).start();

        // start the camera server
        CameraServer.getInstance().startAutomaticCapture();

        // ACTUAL APRIL TAG USAGE - new thread for processing
        VisionThread visionThread = new VisionThread(camera, new VisionPipeline(), pipeline -> {

            //definitions: anything with the word "target" = speaker, and "tag" = apriltags for speaker identification
            // target and tag pose
            Pose2d targetPose = pipeline.getTargetPose();
            Pose2d tagPose = pipeline.getTagPose();

            // display poses
            SmartDashboard.putNumber("Target X", targetPose.getTranslation().getX());
            SmartDashboard.putNumber("Target Y", targetPose.getTranslation().getY());

            // draw target and tag on frame
            Mat frame = pipeline.getOutput();

            // draw target
            Imgproc.circle(frame, new Point(targetPose.getTranslation().getX(), targetPose.getTranslation().getY()), 5, TARGET_COLOR, -1);

            // draw tag
            if (tagPose != null) {
                Point[] tagCorners = new Point[4];
                for (int i = 0; i < 4; i++) {
                    tagCorners[i] = new Point(tagPose.getTranslation().getX() + tagPose.getRotation().getX() * TAG_SIZE * Math.cos(i * Math.PI / 2 + tagPose.getRotation().getRadians()),
                            tagPose.getTranslation().getY() + tagPose.getRotation().getY() * TAG_SIZE * Math.sin(i * Math.PI / 2 + tagPose.getRotation().getRadians()));
                }
                Imgproc.line(frame, tagCorners[0], tagCorners[1], TAG_COLOR, 2);
                Imgproc.line(frame, tagCorners[1], tagCorners[2], TAG_COLOR, 2);
                Imgproc.line(frame, tagCorners[2], tagCorners[3], TAG_COLOR, 2);
                Imgproc.line(frame, tagCorners[3], tagCorners[0], TAG_COLOR, 2);
            }

            // display frame
            CameraServer.getInstance().putVideo("Target Tracker", 640, 480, frame);
        });

        visionThread.start();

          

        // keep program running, idk if this is 100% needed but ill keep it here
        while (true) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
