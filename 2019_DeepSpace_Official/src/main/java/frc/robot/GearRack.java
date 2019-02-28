package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.hal.PDPJNI;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class GearRack extends PIDSubsystem {

    public TalonSRX motor;

    public long encoderZero;

    public String name;
    public float p;
    public float i;
    public float d;
    public float f;
    public int direction;
    public int pdpHandle;
    public int pdpChannel;
    public int limitPort;
    public float gravityAddition;
    public boolean foundPlatform;
    public DigitalInput limit;

    private boolean isEnabled;

    public GearRack(String name, int motorId, float p, float i, float d, float f, int direction, int pdpHandle,
            byte pdpChannel, float maxOutput, float gravityAddition, int limitPort) {
        super(name, p, i, d, f);
        limit = new DigitalInput(limitPort);
        this.name = name;
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.direction = direction;
        this.pdpHandle = pdpHandle;
        this.pdpChannel = pdpChannel;
        this.gravityAddition = gravityAddition;
        motor = new TalonSRX(motorId);
    }

    // What does this do? Doesn't matter
    public void initDefaultCommand() {

    }

    // Return the sensor value
    public double returnPIDInput() {
        return (GetEncoderPosition());
    }

    // Use the pid output, send it to the motor
    public void usePIDOutput(double output) {
        if (isEnabled && !foundPlatform) {
            output += gravityAddition;
        }
        SetMotorSpeed(output * direction);
    }

    public void Write() {

        // double current = PDPJNI.getPDPChannelCurrent((byte) 5, pdpHandle);
        // SmartDashboard.putNumber(name + "Current", current);

        gravityAddition = (float) SmartDashboard.getNumber(name + " Gravity Addition", gravityAddition);
        p = (float) SmartDashboard.getNumber(name + "P", p);
        i = (float) SmartDashboard.getNumber(name + "I", i);
        d = (float) SmartDashboard.getNumber(name + "D", d);
        f = (float) SmartDashboard.getNumber(name + "F", f);

        SmartDashboard.putNumber(name + " Gravity Addition", gravityAddition);
        SmartDashboard.putNumber(name + "P", p);
        SmartDashboard.putNumber(name + "I", i);
        SmartDashboard.putNumber(name + "D", d);
        SmartDashboard.putNumber(name + "F", f);
        // SmartDashboard.putNumber(name, GetEncoderPosition());
        SmartDashboard.putNumber(name + " Output", (float) motor.getMotorOutputPercent());
        // SmartDashboard.putNumber(name + " TEST", 0.5);
        SmartDashboard.putNumber(name + " raw encoder ", GetEncoderPosition());
        // SmartDashboard.putNumber(name + " Speed ",
        // motor.getSelectedSensorVelocity());
        SmartDashboard.putBoolean(name, limit.get());

        PIDController controller = getPIDController();
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
        controller.setF(f);
    }

    public void EnablePID() {
        if (!isEnabled) {
            isEnabled = true;
            enable();
        }
    }

    public void ResetPID() {
        PIDController controller = getPIDController();
        controller.reset();

        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
        controller.setF(f);
    }

    public void DisablePID() {
        if (isEnabled) {
            isEnabled = false;
            disable();
        }
    }

    public void ResetEncoder() {
        encoderZero = motor.getSelectedSensorPosition();
    }

    public long GetEncoderPosition() {
        return (encoderZero - motor.getSelectedSensorPosition()) * direction;
    }

    public void GetLimit() {

    }

    public void SetMotorSpeed(double Speed) {
        if (limit.get() == false || (Speed * direction) < 0) {
            motor.set(ControlMode.PercentOutput, Speed);
        } else {
            System.out.println(name + " Hit Limit ");
            motor.set(ControlMode.PercentOutput, 0);
        }
    }
}
