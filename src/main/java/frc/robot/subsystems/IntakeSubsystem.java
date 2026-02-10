package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class IntakeSubsystem  extends SubsystemBase {
    private final SparkMax m_IntakeMotor1 = new SparkMax(SubsystemConstants.kIntake1CanId, MotorType.kBrushless);
    private final SparkMax m_IntakeMotor2 = new SparkMax(SubsystemConstants.kIntake2CanId, MotorType.kBrushless);
    public IntakeSubsystem() {
        SparkMaxConfig liftConfig =new SparkMaxConfig();
        m_IntakeMotor1.configure(liftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_IntakeMotor2.configure(liftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }
    @Override
    public void periodic() {

    }
    public void setIntakeSpeed(Supplier<Double> speed) {
        //m_IntakeMotorLeft.set(speed.get());
        //m_IntakeMotorRight.set(-1*speed.get());
    }
}
