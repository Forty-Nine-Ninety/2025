package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Ports.VisionConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;

public class VisionSubsystem extends SubsystemBase {
    private SwerveSubsystem m_drivebase;
    private PhotonCamera arducamOne;
    
    private CommandXboxController m_joystick;
    private Transform3d m_pose;
    private double m_lastTargetTime;

    // Pose persistence fields
    private Transform3d m_lastValidPose = null;
    private double m_lastValidPoseTime = 0;
    private static final double POSE_TIMEOUT = 0.3;
    private static final double STOPPING_DISTANCE = 0.3048;

    // Vision toggle
    private volatile boolean m_visionEnabled = false;
    
    // Button edge detection
    private boolean m_prevRightBumper = false;
    
    // Skip frames
    private int m_frameSkip = 0;
    
    // Cached speeds
    private double m_cachedForward = 0;
    private double m_cachedRot = 0;

    public VisionSubsystem(SwerveSubsystem drivebase, PhotonCamera photonCamera, CommandXboxController joystick) {
        m_drivebase = drivebase;
        arducamOne = photonCamera;
        m_joystick = joystick;
        m_pose = new Transform3d();
        m_lastTargetTime = 0;
    }

    // ==================== VISION TOGGLE METHODS ====================

    public void enableVision() {
        m_visionEnabled = true;
        m_lastValidPose = null;
        m_cachedForward = 0;
        m_cachedRot = 0;
        System.out.println("VISION: ENABLED");
    }

    public void disableVision() {
        m_visionEnabled = false;
        m_drivebase.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        m_lastValidPose = null;
        m_pose = new Transform3d();
        m_cachedForward = 0;
        m_cachedRot = 0;
        System.out.println("VISION: DISABLED");
    }

    public void toggleVision() {
        if (m_visionEnabled) {
            disableVision();
        } else {
            enableVision();
        }
    }

    public boolean isVisionEnabled() {
        return m_visionEnabled;
    }

    // ==================== EXISTING METHODS ====================

    public void scanForApriltag() {
        try {
            // USE getLatestResult() instead of getAllUnreadResults()
            // This doesn't accumulate - just gets the most recent frame
            PhotonPipelineResult result = arducamOne.getLatestResult();
            
            if (result != null && result.hasTargets()) {
                m_pose = result.getBestTarget().getBestCameraToTarget();
            } else {
                m_pose = new Transform3d();
            }
        } catch (Exception e) {
            m_pose = new Transform3d();
        }
        // REMOVED: new RumbleCommand(m_joystick); - was creating garbage every frame
    }

    public void update() {
        // ===== BUTTON CHECK - ONLY ONCE PER LOOP =====
        boolean currentRightBumper = m_joystick.getHID().getRightBumper();
        
        if (currentRightBumper && !m_prevRightBumper) {
            System.out.println("TOGGLE PRESSED");
            toggleVision();
        }
        
        m_prevRightBumper = currentRightBumper;
        
        // ===== EXIT IF VISION DISABLED =====
        if (!m_visionEnabled) {
            return;
        }
        
        // ===== SKIP FRAMES =====
        m_frameSkip++;
        if (m_frameSkip < 3) {
            m_drivebase.setChassisSpeeds(new ChassisSpeeds(m_cachedForward, 0, m_cachedRot));
            return;
        }
        m_frameSkip = 0;

        // ===== VISION LOGIC =====
        this.scanForApriltag();
        
        double currentTime = Timer.getFPGATimestamp();
        
        double poseX = m_pose.getX();
        double poseY = m_pose.getY();

        if (poseX != 0 || poseY != 0) {
            m_lastValidPose = m_pose;
            m_lastValidPoseTime = currentTime;
        }

        Transform3d poseToUse = null;
        if (poseX != 0 || poseY != 0) {
            poseToUse = m_pose;
        } else if (m_lastValidPose != null && (currentTime - m_lastValidPoseTime) < POSE_TIMEOUT) {
            poseToUse = m_lastValidPose;
        }

        double forwardSpeed = 0;
        double rotSpeed = 0;

        if (poseToUse != null) {
            double useX = poseToUse.getX();
            double useY = poseToUse.getY();

            double rotGain = Math.max(0.8, Math.min(2.0, 3.0 / useX));
            double rotDeadband = Math.min(0.3, 0.05 + useX * 0.05);
            
            if (Math.abs(useY) > rotDeadband) {
                rotSpeed = useY * rotGain;
                rotSpeed = Math.max(-VisionConstants.maxRotationSpeed, 
                           Math.min(VisionConstants.maxRotationSpeed, rotSpeed));
            }

            double xError = useX - STOPPING_DISTANCE;
            if (xError > 0.05) {
                double baseSpeed = Math.min(xError * 1.5, VisionConstants.maxTranslationSpeed);
                double alignmentFactor = Math.max(0.4, 1.0 - Math.abs(useY) / 0.5);
                forwardSpeed = baseSpeed * alignmentFactor;
            }

            m_lastTargetTime = currentTime;
        }

        // Deadband
        if (Math.abs(forwardSpeed) < 0.05) forwardSpeed = 0;
        if (Math.abs(rotSpeed) < 0.05) rotSpeed = 0;

        // Cache speeds
        m_cachedForward = forwardSpeed;
        m_cachedRot = rotSpeed;

        m_drivebase.setChassisSpeeds(new ChassisSpeeds(forwardSpeed, 0, rotSpeed));
    }
}