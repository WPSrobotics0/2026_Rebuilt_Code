package frc.robot.subsystems;
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

    public ShooterSubsystem() {
        SparkMaxConfig moterConfig =new SparkMaxConfig();
        m_moter.configure(moterConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_speed = () -> 0.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("turret Speed", m_speed.get());
    }

    public void setIntakeSpeed(double speed) {
        m_moter.set(speed);
    }    
}