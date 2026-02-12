package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class TurretSubsystem  extends SubsystemBase {

    private final SparkMax m_TurretMotor = new SparkMax(SubsystemConstants.kTurretId, MotorType.kBrushless);
    private SparkRelativeEncoder m_TurretEncoder = (SparkRelativeEncoder) m_TurretMotor.getEncoder();
    public TurretSubsystem() {
        SparkMaxConfig liftConfig =new SparkMaxConfig();
        m_TurretMotor.configure(liftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    }
    @Override
    public void periodic() {
        
    }
    public void RotateTurret(Supplier<Double> speed) {
        m_TurretMotor.set(speed.get());
        SmartDashboard.putNumber("Spindexer Speed", speed.get());
    }
}
