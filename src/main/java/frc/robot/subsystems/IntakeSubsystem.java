package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class IntakeSubsystem  extends SubsystemBase {

    private final SparkMax m_IntakeLiftMotor = new SparkMax(SubsystemConstants.kIntakeLiftId, MotorType.kBrushless);
    public SparkRelativeEncoder m_IntakeLiftMotorEncoder = (SparkRelativeEncoder) m_IntakeLiftMotor.getEncoder();

    public IntakeSubsystem() {
        SparkMaxConfig liftConfig =new SparkMaxConfig();
        m_IntakeLiftMotor.configure(liftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }

    public void setIntakeSpeed(Supplier<Double> speed) {
        SmartDashboard.putNumber("Intake Speed", speed.get());
    }
    
    public void setRotate(Supplier<Double> speed){
        m_IntakeLiftMotor.set(speed.get());
        SmartDashboard.putNumber("Intake Rotate Speed", speed.get());    
    }
}
