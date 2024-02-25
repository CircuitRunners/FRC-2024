package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    public static void main(String[] args) {
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("photonvision");

        // start cam
        VideoCapture c
      camera.open(0); // 0 is the default camera, change if needed
      camera.set(Videoio.CAP_PROP_FRAME_WIDTH, 640);
      camera.set(Videoio.CAP_PROP_FRAME_HEIGHT, 480);

  

    CameraServer.getInstance().startAutomaticCapture();
    

    VisionThread
    
        //definitions: anything with the word "target" = speaker
        // target and tag pose
        Pose2d tagPose = pipeline.getTagPose();

        SmartDashboard.putNumb
        SmartDashboard.putNumber("Target Y", tagPose.ge

        // draw target and tag on frame
        Mat frame = pipeline.getOutput();

      
      // draw tag
      if (tagPose != null) {

          for (int i =
              tagCorners[i] = new Point(tagPose.getTranslation().getX() + ta
                      tagPose.getTranslation().getY() + tagPose.getRotation(

          Imgproc.line(frame, tagCorn
          Imgproc.line(frame, tagCorner

      }
      
        isplay frame
        raServer.getInstance().putVid
          
              
                  
              
                  
        r
        
        
        
        program running, idk if this is 100% needed but ill keep it here
      e

          Thread.sleep
      } catch (InterruptedException e) {
       

    }

    
    
      
        
      
        
      
    
  