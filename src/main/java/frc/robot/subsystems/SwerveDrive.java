package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private Pose2d m_robotPose;
    private SwerveDriveKinematics m_kinematics;
    
    
    public SwerveDrive() {

    }

    public void move(ChassisSpeeds speeds) {
        // Do some stuff here
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);


    }

    public void moveFieldRelative(double xVelocityMps, double yVelocityMps, double omegaRds) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocityMps, yVelocityMps, omegaRds, m_robotPose.getRotation());
        move(speeds);
    }
}
