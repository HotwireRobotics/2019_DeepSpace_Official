package frc.robot.autostep;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.DriveTrain;

public class NavxTurn extends AutoStep {

    public AHRS navx;
    public float turnDegree;
    public float speed;

    public NavxTurn(DriveTrain driveTrain, AHRS navx, float turnDegree, float speed) {
        super(driveTrain);
        this.navx = navx;
        this.speed = speed;
        this.turnDegree = turnDegree;
    }

    public void Begin() {
        driveTrain.SetLeftSpeed(speed * (turnDegree / Math.abs(turnDegree)));
        driveTrain.SetRightSpeed(-speed * (turnDegree / Math.abs(turnDegree)));
    }

    public void Update() {
        if (turnDegree >= navx.getYaw()) {
            if (Math.abs(navx.getYaw()) > turnDegree) {
                isDone = true;
                driveTrain.SetLeftSpeed(0);
                driveTrain.SetRightSpeed(0);
            }
        }
        if (turnDegree <= navx.getYaw()) {
            if (Math.abs(navx.getYaw()) < turnDegree) {
                isDone = true;
                driveTrain.SetLeftSpeed(0);
                driveTrain.SetRightSpeed(0);
            }
        }

    }
}