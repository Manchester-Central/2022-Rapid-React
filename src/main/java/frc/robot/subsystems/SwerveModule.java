package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveDriveConstants;

public class SwerveModule {
    private WPI_TalonFX m_angle;
    private WPI_TalonFX m_speed;
    private SwerveModuleState m_state;

    public SwerveModule() {
         m_angle = new WPI_TalonFX(SwerveDriveConstants.AngleCanID);
         m_speed = new WPI_TalonFX(SwerveDriveConstants.SpeedCanID);


    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, m_state.angle);
    }

    public SwerveModuleState getstate() {
        return m_state;
    }

    private double speedMpstoTps(double speedMetersPerSecond) {
        return 0;
    }
}
