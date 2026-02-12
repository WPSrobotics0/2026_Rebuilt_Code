package frc.robot.subsystems;
//import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShootingSubsystem  extends SubsystemBase {

    public ShootingSubsystem() {

        //m_navX_Shooter= new AHRS(NavXComType.kUSB1);
    
        //m_navX_Shooter.reset();
       // SparkMaxConfig liftConfig =new SparkMaxConfig();
        //m_ShootingMotor.configure(liftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }
    @Override
    public void periodic() {

       //SmartDashboard.putNumber(getName(), m_navX_Shooter.getRotation2d().getDegrees());
    }

    
}
