package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import java.util.function.Supplier;

//import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.SubsystemConstants;

public class ElevatorSubsystem  extends SubsystemBase {

    private final SparkMax m_IntakeMotorLeft = new SparkMax(55, MotorType.kBrushless);
    //private final SparkMax m_IntakeMotorRight = new SparkMax(SubsystemConstants.kAlgaeLiftCanId, MotorType.kBrushless);
    public ElevatorSubsystem() {
        SparkMaxConfig liftConfig =new SparkMaxConfig();
        m_IntakeMotorLeft.configure(liftConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        //m_IntakeMotorRight.configure(liftConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    }
    @Override
    public void periodic() {

    }
    public void setIntakeSpeed(double speed) {
        m_IntakeMotorLeft.set(speed);
        //m_IntakeMotorRight.set(-1*speed.get());
    }
}
