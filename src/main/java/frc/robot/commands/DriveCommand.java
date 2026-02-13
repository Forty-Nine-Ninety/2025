package frc.robot.commands;

import frc.robot.Constants.Ports.DriveSettings;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.DriveUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
    private final SwerveSubsystem m_drivebase;
    private DoubleSupplier m_translationX, m_translationY, m_angularRotationX;
    private double translationX, translationY, angularRotation;
    private VisionSubsystem m_vision = null;

    public DriveCommand(SwerveSubsystem drivebase) {
        m_drivebase = drivebase;
        addRequirements(drivebase);
    }

    public void setSuppliers(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        m_translationX = translationX;
        m_translationY = translationY;
        m_angularRotationX = angularRotationX;
    }

    public void setSuppliers(double translationX, double translationY, double angularRotation) {
        this.translationX = translationX;
        this.translationY = translationY;
        this.angularRotation = angularRotation;
    }

    public void setVisionSubsystem(VisionSubsystem vision) {
        m_vision = vision;
    }

    @Override
    public void execute() {
        // If vision is enabled, let vision control the robot - don't override it
        if (m_vision != null && m_vision.isVisionEnabled()) {
            return;
        }

        // Make the robot move
        if (m_translationX != null) {
            m_drivebase.drive(new Translation2d(DriveUtil.powCopySign(m_translationX.getAsDouble(), DriveSettings.JOYSTICK_THROTTLE_X_EXPONENT) * DriveSettings.ARCADE_SPEED_X_MULTIPLIER * m_drivebase.getSwerveDrive().getMaximumChassisVelocity(),
                                            DriveUtil.powCopySign(m_translationY.getAsDouble(), DriveSettings.JOYSTICK_THROTTLE_Y_EXPONENT) * DriveSettings.ARCADE_SPEED_Y_MULTIPLIER * m_drivebase.getSwerveDrive().getMaximumChassisVelocity()),
                          DriveUtil.powCopySign(m_angularRotationX.getAsDouble(), DriveSettings.JOYSTICK_TURNING_EXPONENT) * DriveSettings.ARCADE_ROTATIONV2_MULTIPLIER * m_drivebase.getSwerveDrive().getMaximumChassisAngularVelocity(),
                          true);
        } else {
            m_drivebase.drive(new Translation2d(translationX, translationY),
                              angularRotation,
                false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DRIVE COMMAND END");
    }
}