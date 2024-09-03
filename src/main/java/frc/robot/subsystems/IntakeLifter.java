package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.LazyCANSparkMax;

public class IntakeLifter extends SubsystemBase {
    private final LazyCANSparkMax lifterMotor;
    private final ProfiledPIDController lifterMotorPID;
    private final DutyCycleEncoder lifterEncoder;

    private Measure<Angle> desiredAngle = Constants.Intake.Lifter.PhysicalModel.kShooterAngle;

    public IntakeLifter() {
        this.lifterMotor = new LazyCANSparkMax(Constants.Intake.Lifter.kLifterMotorID, MotorType.kBrushless);
        this.lifterMotor.setInverted(true);
        this.lifterMotorPID = Constants.Intake.Lifter.PhysicalModel.kLifterPIDController;
        this.lifterEncoder = new DutyCycleEncoder(Constants.Intake.Lifter.kLifterEncoderPort);
    }

    /**
     * @param angleDeg The angle to set the lifter to
     * @return true if the lifter is within 5 degrees of the target angle
     */
    public void setLifterAngle(Measure<Angle> angleDeg) {
        this.desiredAngle = angleDeg;
    }

    public double getLifterAngleRadians() {
        double angleRadians = (this.lifterEncoder.getAbsolutePosition()-Constants.Intake.Lifter.kLifterEncoderOffset)*2*Math.PI;
        return ((Math.floor(angleRadians * 1000)) / 1000);
    }

    public void stopLifter() {
        this.lifterMotor.set(0);
    }

    public void periodic() {
        double voltage = MathUtil.clamp(this.lifterMotorPID.calculate(getLifterAngleRadians(), this.desiredAngle.in(Radians)), Constants.Intake.Lifter.kMinLifterSpeed, Constants.Intake.Lifter.kMaxLifterVoltage);
        this.lifterMotor.setVoltage(voltage);

        SmartDashboard.putNumber("Intake/LifterAngle", Math.toDegrees(this.getLifterAngleRadians()));
        SmartDashboard.putNumber("Intake/Lifter", this.lifterMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake/SetAngle", this.desiredAngle.in(Degrees));
    }
}
