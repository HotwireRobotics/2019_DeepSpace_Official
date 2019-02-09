package frc.robot;

import edu.wpi.first.hal.PDPJNI;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class GearRack extends PIDSubsystem {

    public DigitalInput limitSwitch;
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

    private boolean isEnabled;

    public GearRack(String name, int motorId, float p, float i, float d, float f, int direction, int pdpHandle,
            byte pdpChannel, int switchID) {
        super(name, p, i, d, f);

        this.name = name;
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.direction = direction;
        this.pdpHandle = pdpHandle;
        this.pdpChannel = pdpChannel;
        limitSwitch = new DigitalInput(switchID);
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
        SetMotorSpeed(output);
    }

    public void Write() {
        // double current = PDPJNI.getPDPChannelCurrent((byte)5, pdpHandle);
        p = (float) SmartDashboard.getNumber(name + "P", p);
        i = (float) SmartDashboard.getNumber(name + "I", i);
        d = (float) SmartDashboard.getNumber(name + "D", d);
        f = (float) SmartDashboard.getNumber(name + "F", f);
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
        SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
        // SmartDashboard.putNumber(name + "Current", current);
        
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

    public void SetMotorSpeed(double Speed) {
        if (limitSwitch.get() == false) {
            motor.set(ControlMode.PercentOutput, Speed * direction);
        } else {
            motor.set(ControlMode.PercentOutput, 0 * direction);
        }
    }

    public long GetEncoderPosition() {
        return (encoderZero - motor.getSelectedSensorPosition()) * direction;
    }
}
