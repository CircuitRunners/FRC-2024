package frc.lib.utils;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldUtil {
  public static final String defaultFieldName = "Field";

  private final Field2d field = new Field2d();

  private FieldUtil(String name) {
    SmartDashboard.putData(name, field);
  }

  public static FieldUtil getField() {
    return new FieldUtil(defaultFieldName);
  }

  public void setObjectGlobalPose(String name, Pose2d pose) {
    if (pose == null) {
      pose = new Pose2d();
    }
    field.getObject(name).setPose(pose);
  }


  public void setObjectGlobalPoses(String name, Pose2d... poses) {
    field.getObject(name).setPoses(poses);
  }

  public void setTrajectory(String name, Trajectory trajectory) {
    if (trajectory == null) {
      trajectory = new Trajectory();
    }
    field.getObject(name).setTrajectory(trajectory);
  }

  public void updateRobotPose(Pose2d pose) {
    field.setRobotPose(pose);
  }

  public void setSwerveRobotPose(Pose2d pose, SwerveModuleState[] states, Translation2d[] translations) {
    updateRobotPose(pose);

    Pose2d[] modulePoses = new Pose2d[states.length];

    for (int i = 0; i < states.length; i++) {
      var translation = translations[i];
      var rotation = states[i].angle;

      modulePoses[i] = pose.transformBy(new Transform2d(translation, rotation));
    }

    setObjectGlobalPoses("RobotSwerveModules", modulePoses);
  }

  public Pose2d getObjectPose(String name) {
    return field.getObject(name).getPose();
  }

  public void removeObject(String name) {
    field.getObject(name).setPoses();
  }
}
