package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Ports.VisionConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
    private static final double STOPPING_DISTANCE = 0.50;

    // Rumble distance threshold
    private static final double RUMBLE_DISTANCE = 0.65;
    
    // Rumble timing
    private static final double RUMBLE_ON_TIME = 2.0;   // Rumble for 2 seconds
    private static final double RUMBLE_OFF_TIME = 1.0;  // Off for 1 second
    private double m_rumbleCycleStart = 0;
    private boolean m_inRumbleZone = false;

    // Vision toggle
    private volatile boolean m_visionEnabled = false;
    
    // Button edge detection
    private boolean m_prevRightBumper = false;
    
    // Skip frames
    private int m_frameSkip = 0;
    
    // Cached speeds
    private double m_cachedForward = 0;
    private double m_cachedStrafe = 0;
    private double m_cachedRot = 0;

    // Manual override scaling (0.3 = 30% of normal joystick speed)
    private static final double MANUAL_OVERRIDE_SCALE = 0.3;
    private static final double JOYSTICK_DEADBAND = 0.1;

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
        m_cachedStrafe = 0;
        m_cachedRot = 0;
        m_inRumbleZone = false;
        System.out.println("VISION: ENABLED");
    }

    public void disableVision() {
        m_visionEnabled = false;
        m_drivebase.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        m_lastValidPose = null;
        m_pose = new Transform3d();
        m_cachedForward = 0;
        m_cachedStrafe = 0;
        m_cachedRot = 0;
        // Stop rumble when vision disabled
        setRumble(0);
        m_inRumbleZone = false;
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

    // ==================== RUMBLE ====================

    private void setRumble(double intensity) {
        m_joystick.getHID().setRumble(RumbleType.kLeftRumble, intensity);
        m_joystick.getHID().setRumble(RumbleType.kRightRumble, intensity);
    }

    private void updateRumble(double distance) {
        double currentTime = Timer.getFPGATimestamp();
        
        if (distance <= RUMBLE_DISTANCE) {
            // Just entered rumble zone - start cycle
            if (!m_inRumbleZone) {
                m_inRumbleZone = true;
                m_rumbleCycleStart = currentTime;
                setRumble(1.0);
                System.out.println("RUMBLE: STARTED");
            }
            
            // Calculate where we are in the cycle
            double cycleTime = currentTime - m_rumbleCycleStart;
            double cyclePeriod = RUMBLE_ON_TIME + RUMBLE_OFF_TIME;
            double positionInCycle = cycleTime % cyclePeriod;
            
            if (positionInCycle < RUMBLE_ON_TIME) {
                setRumble(1.0);  // Rumble ON
            } else {
                setRumble(0);   // Rumble OFF
            }
        } else {
            // Outside rumble zone
            if (m_inRumbleZone) {
                m_inRumbleZone = false;
                setRumble(0);
                System.out.println("RUMBLE: STOPPED");
            }
        }
    }

    // ==================== MANUAL OVERRIDE ====================

    /**
     * Get field-relative manual override speeds.
     * Converts joystick input to field-relative using gyro heading.
     */
    private ChassisSpeeds getFieldRelativeManualSpeeds() {
        double joystickX = MathUtil.applyDeadband(-m_joystick.getLeftY(), JOYSTICK_DEADBAND);
        double joystickY = MathUtil.applyDeadband(-m_joystick.getLeftX(), JOYSTICK_DEADBAND);
        double joystickRot = MathUtil.applyDeadband(-m_joystick.getRightX(), JOYSTICK_DEADBAND);
        
        // Scale down for fine control
        double maxSpeed = 4.5 * MANUAL_OVERRIDE_SCALE;
        double maxRot = 4.5 * MANUAL_OVERRIDE_SCALE;
        
        double fieldX = joystickX * maxSpeed;
        double fieldY = joystickY * maxSpeed;
        double rot = joystickRot * maxRot;
        
        // Convert field-relative to robot-relative using gyro
        Rotation2d gyroAngle = m_drivebase.getHeading();
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(fieldX, fieldY, rot);
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, gyroAngle);
        
        return robotSpeeds;
    }

    // ==================== EXISTING METHODS ====================

    public void scanForApriltag() {
        try {
            PhotonPipelineResult result = arducamOne.getLatestResult();
            
            if (result != null && result.hasTargets()) {
                m_pose = result.getBestTarget().getBestCameraToTarget();
            } else {
                m_pose = new Transform3d();
            }
        } catch (Exception e) {
            m_pose = new Transform3d();
        }
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
        
        // ===== GET FIELD-RELATIVE MANUAL OVERRIDE =====
        ChassisSpeeds manualSpeeds = getFieldRelativeManualSpeeds();
        double manualForward = manualSpeeds.vxMetersPerSecond;
        double manualStrafe = manualSpeeds.vyMetersPerSecond;
        double manualRot = manualSpeeds.omegaRadiansPerSecond;
        
        // ===== SKIP FRAMES =====
        m_frameSkip++;
        if (m_frameSkip < 3) {
            m_drivebase.setChassisSpeeds(new ChassisSpeeds(
                m_cachedForward + manualForward, 
                m_cachedStrafe + manualStrafe, 
                m_cachedRot + manualRot
            ));
            return;
        }
        m_frameSkip = 0;

        // ===== VISION LOGIC =====
        this.scanForApriltag();
        
        double currentTime = Timer.getFPGATimestamp();
        
        double poseX = m_pose.getX();
     //   double poseX = m_pose.getX();

        double poseY = m_pose.getY();
       // double poseY = m_pose.getY();

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

        double visionForward = 0;

        double visionRot = 0;

        if (poseToUse != null) {
            double useX = poseToUse.getX();
            double useY = poseToUse.getY();

            // ===== PERIODIC RUMBLE WHEN CLOSE =====
            updateRumble(useX);

            double rotGain = Math.max(0.8, Math.min(2.0, 3.0 / useX));
            double rotDeadband = Math.min(0.3, 0.05 + useX * 0.05);
            
            if (Math.abs(useY) > rotDeadband) {
                visionRot = useY * rotGain;
                visionRot = Math.max(-VisionConstants.maxRotationSpeed, 
                           Math.min(VisionConstants.maxRotationSpeed, visionRot));
            }

            double xError = useX - STOPPING_DISTANCE;
            if (xError > 0.05) {
                double baseSpeed = Math.min(xError * 1.5, VisionConstants.maxTranslationSpeed);
                double alignmentFactor = Math.max(0.4, 1.0 - Math.abs(useY) / 0.5);
                visionForward = baseSpeed * alignmentFactor;
            }

            m_lastTargetTime = currentTime;
        } else {
            // No target - stop rumble
            updateRumble(999);  // Pass large distance to stop rumble
        }

        // Deadband on vision speeds
        if (Math.abs(visionForward) < 0.05) visionForward = 0;
        if (Math.abs(visionRot) < 0.05) visionRot = 0;

        // Cache vision speeds
        m_cachedForward = visionForward;
        m_cachedStrafe = 0;
        m_cachedRot = visionRot;

        // ===== COMBINE VISION (robot-relative) + MANUAL (field-relative converted to robot) =====
        m_drivebase.setChassisSpeeds(new ChassisSpeeds(
            visionForward + manualForward, 
            manualStrafe, 
            visionRot + manualRot
        ));
    }
}