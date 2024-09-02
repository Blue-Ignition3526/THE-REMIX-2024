package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.LazyCANSparkMax;

public class IntakeRollers extends SubsystemBase {
    private final LazyCANSparkMax rollersMotor;
    private final DigitalInput pieceSwitch;

    private boolean isIntaking = false;

    public IntakeRollers() {
        this.rollersMotor = new LazyCANSparkMax(Constants.Intake.Rollers.kintakeRollersMotorID, MotorType.kBrushless);
        this.pieceSwitch = new DigitalInput(Constants.Intake.Rollers.kPieceSwitchPort);
    }

    // * Idle modes (for not damaging gearbox)
    public void setRollersCoast() {
        this.rollersMotor.setIdleMode(IdleMode.kCoast);
        SmartDashboard.putBoolean("Intake/Brake", false);
    }

    public void setRollersBrake() {
        SmartDashboard.putBoolean("Intake/Brake", true);
        this.rollersMotor.setIdleMode(IdleMode.kBrake);
    }

    // * Piece switch
    public boolean hasPiece() {
        return !this.pieceSwitch.get();
    }

    // * Speed setters
    public void setRollersSpeed(double speed) {
        this.isIntaking = speed > 0;
        this.rollersMotor.set(speed);
        SmartDashboard.putBoolean("Intake/Intaking", this.isIntaking);
    }

    public void setRollersOut() {
        this.setRollersSpeed(Constants.Intake.Rollers.kIntakeOutSpeed);
    }

    public void setRollersIn() {
        this.setRollersSpeed(Constants.Intake.Rollers.kIntakeInSpeed);
    }

    public void giveToShooter() {
        this.setRollersSpeed(Constants.Intake.Rollers.kGiveToShooterSpeed);
    }

    public void setRollersHold() {
        this.setRollersSpeed(Constants.Intake.Rollers.kIntakeHoldSpeed);
    }

    public void stop() {
        this.setRollersSpeed(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake/HasPiece", this.hasPiece());
    }
}