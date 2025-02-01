package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import java.lang.Math;

// proof of concept subsystem
public class MotorSubsystem extends SubsystemBase{
    private double PIDFeedback=0;
    private double kP = 1.2; //calibrate using Ziegler-Nickels Heuristic 1.2=ku
    private double kI = 0.0;
    private double kD = 0.0;
    private double TemporaryN = 0;
    private boolean MotorToggle = false;
    double encoder;

    // private final SparkMax motor = new SparkMax(11, MotorType.kBrushless);
    private final TalonFX motor = new TalonFX(0);
    // RelativeEncoder encoder = motor.getEncoder();
    PIDController pid = new PIDController(kP, kI, kD);

    public MotorSubsystem() {

    }

    @Override
    public void periodic() {
        double encoder = motor.getPosition().getValue().magnitude();
        pid.setSetpoint(TemporaryN);
        PIDFeedback = pid.calculate(encoder);
        if (MotorToggle) {
            adjustToSetPoint();
        }
        if (encoder == TemporaryN) {
            MotorToggle = false;
        }
    }

    public void addNtoRotate(Double N) {
        TemporaryN = TemporaryN + N;
    }

    public void runMotor() {
        MotorToggle = true;
    }

    public void resetEncoder() {
        motor.setPosition(0.0);
    }

    public void setPIDTolerance(double value) {
        pid.setTolerance(value);
    }

    private double SigmoidAdjustment(double value) {
        double num = value * 0.5;
        double denom = 1 + Math.abs(value);
        return num/denom;
    }

    private void adjustToSetPoint() {
        motor.set(SigmoidAdjustment(PIDFeedback));
    }

    public void stopMotor() {
        motor.stopMotor();
        MotorToggle = false;
    }

    public double getEncoderPosition() {
        return encoder;
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getSetpoint() {
       return pid.getSetpoint();
    }

    public double getPIDFeedback() {
        return PIDFeedback;
    }

    public double getSigmoid() {
        return SigmoidAdjustment(PIDFeedback);
    }

    public double getkP() {
        return kP;
    }

    public void increasekP() {
        kP= kP + 0.1;
        pid.setP(kP);
    }

    public double getkI() {
        return kI;
    }

    public void increasekI() {
        kI= kI + 0.1;
        pid.setI(kI);
    }

    public double getkD() {
        return kD;
    }

    public void increasekD() {
        kD= kD + 0.1;
        pid.setD(kD);
    }
}
