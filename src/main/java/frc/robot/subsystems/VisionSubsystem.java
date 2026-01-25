package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Ports.DriveSettings;
import frc.robot.Constants.Ports.VisionConstants;
import frc.robot.DriveUtil;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.RumbleCommandHelper;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.VisionDriveCommand;
import frc.robot.commands.auto.AutoVisionCommand;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    private SwerveSubsystem m_drivebase;
    private VisionDriveCommand m_visionDrive;
    private DriveCommand m_driveCommand;
    private PhotonCamera arducamOne;
    
    private List<PhotonPipelineResult> results;
    private PhotonTrackedTarget currentTarget;
    private RumbleCommand m_rumble;
    private CommandXboxController m_joystick;
    private Transform3d m_pose;
    private double m_lastTargetTime;
    private double[] m_speeds;
    private Transform3d m_prevPose;
    private boolean driving = false;

    // Pose persistence fields
    private Transform3d m_lastValidPose = null;
    private double m_lastValidPoseTime = 0;
    private static final double POSE_TIMEOUT = 0.3;
    private static final double STOPPING_DISTANCE = 0.3048; // 1 foot in meters

    public VisionSubsystem(SwerveSubsystem drivebase, PhotonCamera photonCamera, CommandXboxController joystick) {
        m_drivebase = drivebase;
        arducamOne = photonCamera;
        m_rumble = new RumbleCommand(joystick);
        m_joystick = joystick;
        m_pose = new Transform3d();
        m_lastTargetTime = 0;
        m_driveCommand = new DriveCommand(m_drivebase);
        m_speeds = new double[3];
    }

    public void scanForApriltag() {
        results = arducamOne.getAllUnreadResults();
        
        if (results.size() != 0 && results != null) {
            PhotonPipelineResult result = results.get(results.size() - 1);
            if (result.hasTargets() && !driving) {
                m_pose = result.getBestTarget().getBestCameraToTarget();
            } else {
                m_pose = new Transform3d();
            }
        } else {
            m_pose = new Transform3d();
        }
        results.clear();
    }

    public void update() {
        this.scanForApriltag();
        double currentTime = Timer.getFPGATimestamp();
        
        double poseX = m_pose.getX();
        double poseY = m_pose.getY();

        // Store valid pose when we see one
        if (poseX != 0 || poseY != 0) {
            m_lastValidPose = m_pose;
            m_lastValidPoseTime = currentTime;
        }

        // Determine which pose to use
        Transform3d poseToUse = null;
        if (poseX != 0 || poseY != 0) {
            poseToUse = m_pose;
        } else if (m_lastValidPose != null && (currentTime - m_lastValidPoseTime) < POSE_TIMEOUT) {
            poseToUse = m_lastValidPose;
        }

        // Reset all speeds
        double forwardSpeed = 0;
        double strafeSpeed = 0;
        double rotSpeed = 0;

        if (poseToUse != null) {
            double useX = poseToUse.getX();
            double useY = poseToUse.getY();

            // ROTATION: Always turn toward tag
            if (Math.abs(useY) > 0.05) {
                rotSpeed = useY * 2.0;
                if (rotSpeed > VisionConstants.maxRotationSpeed) {
                    rotSpeed = VisionConstants.maxRotationSpeed;
                } else if (rotSpeed < -VisionConstants.maxRotationSpeed) {
                    rotSpeed = -VisionConstants.maxRotationSpeed;
                }
            }

            // FORWARD: Always drive toward tag (but slower when misaligned)
            double xError = useX - STOPPING_DISTANCE;
            if (xError > 0.05) {
                // Calculate base forward speed
                double baseSpeed = Math.min(xError * 1.5, VisionConstants.maxTranslationSpeed);
                
                // Scale down speed when misaligned (but minimum 40% speed)
                double alignmentFactor = Math.max(0.4, 1.0 - Math.abs(useY) / 0.5);
                forwardSpeed = baseSpeed * alignmentFactor;
            }

            // No strafing
            strafeSpeed = 0;

            System.out.println("STATE: DRIVING+TURNING | useX=" + useX + " useY=" + useY + " fwd=" + forwardSpeed + " rot=" + rotSpeed);

            m_lastTargetTime = currentTime;
        } else {
            System.out.println("STATE: NO TARGET");
        }

        // Deadband
        if (Math.abs(forwardSpeed) < 0.05) forwardSpeed = 0;
        if (Math.abs(strafeSpeed) < 0.05) strafeSpeed = 0;
        if (Math.abs(rotSpeed) < 0.05) rotSpeed = 0;

        System.out.println("FINAL: fwd=" + forwardSpeed + " strafe=" + strafeSpeed + " rot=" + rotSpeed);

        m_drivebase.setChassisSpeeds(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotSpeed));
    }
}