package lib.BlueShift.control.speedset;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicSpeedset {
    private ChassisSpeeds speeds;

    public HolonomicSpeedset() {}

    public void set(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    public ChassisSpeeds get() {
        return speeds;
    }
}
