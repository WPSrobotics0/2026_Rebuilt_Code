package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import com.revrobotics.ResetMode;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class ShootingSubsystem  extends SubsystemBase {
<<<<<<< HEAD
    private final SparkMax m_ShootingMotor = new SparkMax(SubsystemConstants.kShooterCanId, MotorType.kBrushless);
    //private AHRS m_navX_Shooter;
    //Supplier<Double> m_ticks;
=======
    //private final SparkMax m_ShootingMotor = new SparkMax(SubsystemConstants.kAlgaeLiftCanId, MotorType.kBrushless);
>>>>>>> 8c09a8189a6ae898d4b3c34a6311a49a6da40121

    public ShootingSubsystem() {

        //m_navX_Shooter= new AHRS(NavXComType.kUSB1);
    
        //m_navX_Shooter.reset();
        SparkMaxConfig liftConfig =new SparkMaxConfig();
        //m_ShootingMotor.configure(liftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }
    @Override
    public void periodic() {

       //SmartDashboard.putNumber(getName(), m_navX_Shooter.getRotation2d().getDegrees());
    }
<<<<<<< HEAD
    public void getDegrees(Supplier<Double> ticks) {
        //m_ShootingMotor.set(speed.get());
            //if(ticks.get()%2==1){
                //SmartDashboard.putNumber(getName(), m_navX_Shooter.getRotation2d().getDegrees());
        //}
           
=======
    public void setShootingSpeed(Supplier<Double> speed) {
        //m_ShootingMotor.set(speed.get());
        
>>>>>>> 8c09a8189a6ae898d4b3c34a6311a49a6da40121
    }
}
