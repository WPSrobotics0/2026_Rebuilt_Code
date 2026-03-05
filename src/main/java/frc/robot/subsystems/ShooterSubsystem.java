package frc.robot.subsystems;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class ShooterSubsystem  extends SubsystemBase {
    private final SparkMax m_moter = new SparkMax(SubsystemConstants.kShooterId, MotorType.kBrushless);
    private Supplier<Double> m_speed;
    //private final SparkMax m_IntakeMotorRight = new SparkMax(SubsystemConstants.kAlgaeLiftCanId, MotorType.kBrushless);
    public ShooterSubsystem() {
        SparkMaxConfig moterConfig =new SparkMaxConfig();
        
        m_moter.configure(moterConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_speed = () -> 0.0;
        //m_IntakeMotorRight.configure(liftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("turret Speed", m_speed.get());
        
    }
    public void setIntakeSpeed(double speed) {
        m_speed = () -> speed;
        m_moter.set(speed);
        //m_moter.set(m_speed.get());
        //m_IntakeMotorRight.set(-1*speed.get());
    }

    
}
