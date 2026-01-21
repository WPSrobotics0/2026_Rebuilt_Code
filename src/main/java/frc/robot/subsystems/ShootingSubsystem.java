package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class ShootingSubsystem  extends SubsystemBase {
    private final SparkMax m_ShootingMotor = new SparkMax(SubsystemConstants.kAlgaeLiftCanId, MotorType.kBrushless);

    public ShootingSubsystem() {
        SparkMaxConfig liftConfig =new SparkMaxConfig();
        m_ShootingMotor.configure(liftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }
    @Override
    public void periodic() {

    }
    public void setShootingSpeed(Supplier<Double> speed) {
        m_ShootingMotor.set(speed.get());
        
    }
}
