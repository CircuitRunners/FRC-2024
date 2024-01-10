package frc.lib.utils;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;

public class PathPlannerUtil {
  private static final NetworkTableInstance kInstance = NetworkTableInstance.getDefault();

  private static final DoubleArraySubscriber kTargetPoseSub = kInstance.getDoubleArrayTopic("/PathPlanner/targetPose")
      .subscribe(new double[] { 0.0, 0.0, 0.0 });

  public static List<String> getExistingPaths() {
    var path = Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner");
    try (Stream<Path> stream = Files.walk(path)) {
      return stream
          .filter(x -> getFileExtension(x).equals(".auto"))
          .map(x -> getFileStem(x))
          .toList();
    } catch (IOException e) {
      return Collections.emptyList();
    }
  }

  private static String getFileStem(Path path) {
    try {
      String name = path.getFileName().toString();
      return name.substring(0, name.lastIndexOf("."));
    } catch (Exception e) {
      return "";
    }
  }

  private static String getFileExtension(Path path) {
    try {
      String name = path.getFileName().toString();
      return name.substring(name.lastIndexOf("."));
    } catch (Exception e) {
      return "";
    }
  }

  public static Pose2d getCurrentTargetPose() {
    var arr = kTargetPoseSub.get();
    double x = arr[0];
    double y = arr[1];
    Rotation2d rot = Rotation2d.fromRadians(arr[2]);

    return new Pose2d(x, y, rot);
  }
}
