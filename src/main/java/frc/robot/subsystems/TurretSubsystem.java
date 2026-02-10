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

public class TurretSubsystem  extends SubsystemBase {
    private final SparkMax m_moter = new SparkMax(41, MotorType.kBrushless);
    private Supplier<Double> m_speed;
    //private final SparkMax m_IntakeMotorRight = new SparkMax(SubsystemConstants.kAlgaeLiftCanId, MotorType.kBrushless);
    public TurretSubsystem() {
        SparkMaxConfig moterConfig =new SparkMaxConfig();
        m_moter.configure(moterConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_speed = () -> 0.0;
        //m_IntakeMotorRight.configure(liftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", m_speed.get());
    }
    public void setIntakeSpeed(Supplier<Double> speed) {
        m_speed = () -> speed.get();
        m_moter.set(speed.get());
        //m_IntakeMotorRight.set(-1*speed.get());
    }

    
}
