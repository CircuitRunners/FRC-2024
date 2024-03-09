package frc.robot;

import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Vision {
  public static final record VisionMeasurement (Pose2d pose, double timestamp, Matrix<N3, N1> stdDev) {}
  private final PhotonCamera frontRightCam = new PhotonCamera("frontRight");
  private final PhotonCamera frontLeftCam = new PhotonCamera("frontLeft");

  private final PhotonPoseEstimator frontRightPoseEstimator = new PhotonPoseEstimator(Constants.Vision.fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontRightCam, Constants.Vision.frontRightCamTransform);
  private final PhotonPoseEstimator frontLeftPoseEstimator = new PhotonPoseEstimator(Constants.Vision.fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontLeftCam, Constants.Vision.frontLeftCamTransform);

  private final Consumer<VisionMeasurement> visionMeasurementConsumer;

  public Vision(Consumer<VisionMeasurement> visionMeasurementConsumer){
    this.visionMeasurementConsumer = visionMeasurementConsumer;
  }

  public void run(){
    var frontRightUpdate = frontRightPoseEstimator.update();
    var frontLeftUpdate = frontLeftPoseEstimator.update();

    if(frontRightUpdate.isPresent()){
      var estimate = frontRightUpdate.get();
      visionMeasurementConsumer.accept(
        new VisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, VecBuilder.fill(0.9, .9, .9))
      );
    }

    if(frontLeftUpdate.isPresent()){
      var estimate = frontLeftUpdate.get();
      visionMeasurementConsumer.accept(
        new VisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, VecBuilder.fill(0.9, .9, .9))
      );
    }
  }
}
