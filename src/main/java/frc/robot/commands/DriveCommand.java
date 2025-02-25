package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.Constants.Ports.DriveSettings;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.DriveUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command{
    private final SwerveSubsystem m_drivebase;
    private DoubleSupplier m_translationX, m_translationY, m_angularRotationX;

    public DriveCommand(SwerveSubsystem drivebase) {
        m_drivebase = drivebase;
        addRequirements(drivebase);
    }

    public void setSuppliers(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        m_translationX = translationX;
        m_translationY = translationY;
        m_angularRotationX = angularRotationX;
    }

    @Override
    public void execute() {
        // Make the robot move
        m_drivebase.drive(new Translation2d(DriveUtil.powCopySign(m_translationX.getAsDouble(), DriveSettings.JOYSTICK_THROTTLE_X_EXPONENT) * DriveSettings.ARCADE_SPEED_X_MULTIPLIER * m_drivebase.getSwerveDrive().getMaximumChassisVelocity(),
                                            DriveUtil.powCopySign(m_translationY.getAsDouble(), DriveSettings.JOYSTICK_THROTTLE_Y_EXPONENT) * DriveSettings.ARCADE_SPEED_Y_MULTIPLIER * m_drivebase.getSwerveDrive().getMaximumChassisVelocity()),
                          DriveUtil.powCopySign(m_angularRotationX.getAsDouble(), DriveSettings.JOYSTICK_TURNING_EXPONENT) * DriveSettings.ARCADE_ROTATIONV2_MULTIPLIER * m_drivebase.getSwerveDrive().getMaximumChassisAngularVelocity(),
                          true);
    }
}
