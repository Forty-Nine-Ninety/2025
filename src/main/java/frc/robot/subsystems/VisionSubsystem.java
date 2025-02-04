import edu.wpi.first.photonvision.PhotonCamera;
import edu.wpi.first.photonvision.PhotonPipelineResult;
import edu.wpi.first.photonvision.PhotonPoseEstimator;

public class VisionSubsystem {
    private PhotonCamera arducamOne;
    // private PhotonCamera arducamTwo;
    private PhotonPoseEstimator poseEstimator;

    public VisionSubsystem(){
        arducamOne = new PhotonCamera("arducamOne");
        // arducammTwo = new PhotonCamera("arducamTwo");
    }

    public void update(){
        PhotonPipelineResult result = camera.getLatestResult();

        if(result.hasTargets()) {
            poseEstimator.update();
        }
    }
}