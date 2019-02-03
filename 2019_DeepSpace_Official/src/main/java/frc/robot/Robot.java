package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import jdk.internal.net.http.common.Demand;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.hal.PDPJNI;

public class Robot extends TimedRobot {

	// Sensors
	public AHRS navx = new AHRS(SPI.Port.kMXP);
	public Ultrasonic ultrasonic = new Ultrasonic(0, 1);
	public AnalogPotentiometer pot = new AnalogPotentiometer(0);
	// public DigitalInput limitSwich = new DigitalInput(9);

	// Drivetrain
	public JoshMotorControllor badMotor = new JoshMotorControllor(5, 0.8f, false);
	public DriveTrain driveTrain = new DriveTrain(0, 1, 2, 3, navx);

	// neumatics
	public DoubleSolenoid hatch = new DoubleSolenoid(4, 5);
	public DoubleSolenoid diskBrake = new DoubleSolenoid(2,3);
	public boolean buttonReleased;
	public boolean hatchReleased;

	// Joysticks
	public Joystick driver;
	public Joystick operator;
	public Joystick debug;

	// Arm
	public TalonSRX armLeft = new TalonSRX(3);
	public TalonSRX armRight = new TalonSRX(4);

	// Intake

	public TalonSRX intake = new TalonSRX(1);

	// PID Controllers
	public int pdpHandle;
	public GearRack gearRackFrontOne = new GearRack("Front Gear Rack", 5, 0.01f, 0.0f, 0.0f, 0.0f, 1, pdpHandle,
			(byte) 6, 4);
	public GearRack gearRackBackOne = new GearRack("Back Gear Rack One", 7, 0.01f, 0.0f, 0.0f, 0.0f, 1, pdpHandle,
			(byte) 5, 6);
	public GearRack gearRackBackTwo = new GearRack("Back Gear Rack Two", 8, 0.01f, 0.0f, 0.0f, 0.0f, -1, pdpHandle,
			(byte) 4, 7);
	public GearRack gearRackFrontTwo = new GearRack("Front Gear Rack Two", 6, 0.01f, 0.0f, 0.0f, 0.0f, -1, pdpHandle,
			(byte) 4, 5);

	// Ramp timer
	public float timerDelaySeconds = 0.1f;

	public float rampLengthSeconds = 5;
	public float rampCurrent = 0.0f;
	public float rampStep;

	public float rampTargetPoint = 0.0f;
	public Timer pidTimer = new Timer();
	public boolean timerTrue;

	public boolean hitTarget = false;

	// Auto

	public void robotInit() {

		pidTimer.start();
		HatchRelease();
	}

	public void disabledPeriodic() {
		// Controllers
		driver = new Joystick(0);
		operator = new Joystick(1);
	}

	public void autonomousInit() {

	}

	public void autonomousPeriodic() {

		UpdateMotors();
	}

	public void teleopInit() {

		ultrasonic.setAutomaticMode(true);

		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);

		// pdpHandle = PDPJNI.initializePDP(0);
		// rampCurrent = 0f;
	}

	public void teleopPeriodic() {

		// if (pidTimer.get() >= timerDelaySeconds) {
		// pidTimer.reset();
		// if (timerTrue) {
		// if (rampTargetPoint > rampCurrent) {
		// rampCurrent += rampStep;
		// if (rampCurrent > rampTargetPoint) {
		// rampCurrent = rampTargetPoint;
		// }
		// } else if (rampTargetPoint < rampCurrent) {
		// rampCurrent -= rampStep;
		// if (rampCurrent < rampTargetPoint) {
		// rampCurrent = rampTargetPoint;
		// }
		// }
		// }
		// }

		// gearRackFrontOne.Write();
		// gearRackBackTwo.Write();
		// gearRackBackOne.Write();

		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");

		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);

		// Driver Controls

		if (operator.getRawButton(5) || operator.getRawButton(6)) {

			driver.setRumble(RumbleType.kLeftRumble, 1);
			boolean isPlacing = operator.getRawButton(5);
			float approachSpeed = 0.45f;
			float turnSpeed = 0.5f;
			float buffer = 4.0f;
			float approachtarget = 2.1f;
			float reverseSpeed = -0.5f;

			if (!hitTarget) {
				if (x >= buffer) {

					if (isPlacing) {
						HatchHold();
					} else {
						HatchRelease();
					}

					driveTrain.SetLeftSpeed(turnSpeed);
					driveTrain.SetRightSpeed(-turnSpeed);
				} else if (x <= -buffer) {

					if (isPlacing) {
						HatchHold();
					} else {
						HatchRelease();
					}

					driveTrain.SetLeftSpeed(-turnSpeed);
					driveTrain.SetRightSpeed(turnSpeed);
				} else {
					driveTrain.SetLeftSpeed(0f);
					driveTrain.SetRightSpeed(0f);
					if (area < approachtarget) {

						if (isPlacing) {
							HatchHold();
						} else {
							HatchRelease();
						}

						driveTrain.SetLeftSpeed(approachSpeed);
						driveTrain.SetRightSpeed(approachSpeed);
					} else {
						hitTarget = true;
					}
				}
			} else {
				if (isPlacing) {
					HatchRelease();
				} else {
					HatchHold();
				}
			}
		} else {

			driver.setRumble(RumbleType.kLeftRumble, 0);
			hitTarget = false;
			ControllerDrive();

			// gearRackBackTwo.UpdateVelocity();
			// Front Gear Rack 3 1/2 times

			// SmartDashboard.putNumber("Ramp Target", rampTargetPoint);
			// SmartDashboard.putNumber("Ramp Value", rampCurrent);

			// float backMaxOutput = 0.3f;
			// gearRackBackOne.setOutputRange(0.0f, backMaxOutput);
			// gearRackBackTwo.setOutputRange(0.0f, backMaxOutput);

			// if (driver.getRawButton(1)) {

			// UpdateRampTarget(67864);
			// timerTrue = true;
			// rampRate

			// gearRackBackOne.EnablePID();
			// gearRackBackOne.setSetpoint(rampCurrent);
			// gearRackBackTwo.EnablePID();
			// gearRackBackTwo.setSetpoint(rampCurrent);

			// } else if (driver.getRawButton(4)) {
			// UpdateRampTarget(0);
			// timerTrue = true;
			// gearRackBackOne.EnablePID();
			// gearRackBackOne.setSetpoint(rampCurrent);
			// gearRackBackTwo.EnablePID();
			// gearRackBackTwo.setSetpoint(rampCurrent);

			// } else {
			// timerTrue = false;
			// gearRackBackTwo.DisablePID();
			// gearRackBackTwo.DisablePID();
			// gearRackBackOne.DisablePID();
			// }

			// if (driver.getRawButton(8)) {
			// gearRackBackTwo.ResetEncoder();
			// gearRackBackOne.ResetEncoder();
			// }

			/*
			 * if (driver.getRawButton(2)) {
			 * 
			 * //gearRackFrontOne.EnablePID();
			 * 
			 * if (driver.getRawAxis(2) > 0.5) {
			 * 
			 * gearRackBackTwo.setSetpoint(speedTarget);
			 * 
			 * // gearRackFrontOne.setOutputRange(0, 1.0); gearRackBackOne.setOutputRange(0,
			 * 1.0); gearRackBackTwo.setOutputRange(0, 1.0); } else {
			 * 
			 * gearRackBackTwo.motor.set(ControlMode.PercentOutput, 0.5f);
			 * gearRackFrontOne.motor.set(ControlMode.PercentOutput, 0.5f);
			 * 
			 * //gearRackFrontOne.setSetpoint(-speedTarget);
			 * 
			 * //gearRackFrontOne.setOutputRange(-1, 0); gearRackBackOne.setOutputRange(-1,
			 * 0); gearRackBackTwo.setOutputRange(-1, 0); } } else {
			 * //gearRackFrontOne.motor.set(ControlMode.PercentOutput, 0.0f);
			 * 
			 * //gearRackFrontOne.DisablePID(); //gearRackBackOne.DisablePID();
			 * //gearRackBackTwo.DisablePID(); }
			 */

			// if (driver.getRawButton(4)) {
			// climbTarget = 0;
			// }

			// if (driver.getRawButton(8)) {
			// gearRackBackOne.ResetEncoder();
			// gearRackBackTwo.ResetEncoder();
			// gearRackFrontOne.ResetEncoder();
			// }

			// Operator Controls
			// if (operator.getRawButton(4)) {
			// Climb(0.4f);
			// } else if (operator.getRawButton(1)) {
			// Climb(-0.4f);
			// }

			// Intake

			if (operator.getRawButton(1)) {
				Intake(0.9f);
			} else {
				if (operator.getRawButton(4)) {
					Outtake(0.9f);
				} else {
					Outtake(0.0f);
				}
			}

			if(operator.getRawButton(7)){
				diskBrake.set(DoubleSolenoid.Value.kForward);
			}else{
				diskBrake.set(DoubleSolenoid.Value.kReverse);
			}

			// System.out.println("Ultrasonic "+ ultrasonic.getRangeInches());

			System.out.println("Pot:"  + pot.get());

			SmartDashboard.putNumber("ultrasonic_down", ultrasonic.getRangeInches());

			// System.out.println(switchOne.get());
			// SmartDashboard.putNumber("navx_value", navx.getYaw());
			// SmartDashboard.putNumber("pot_value", );

			// Hatch

			if (operator.getRawButton(3)) {
				if (buttonReleased == true) {
					buttonReleased = false;
					if (hatchReleased) {
						HatchHold();
					} else {
						HatchRelease();
					}
				}
			} else {
				buttonReleased = true;
			}

			// Arm
			if (operator.getRawButton(2)) {
			 ArmMove(0.4f);
			 }else{
			ArmMove(0.0f);
			 }
		}
			
		UpdateMotors();
	}

	public void testInit() {
		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);
	}

	public void testPeriodic() {

		gearRackFrontOne.Write();

		if (driver.getRawButton(2)) {
			gearRackFrontOne.motor.set(ControlMode.PercentOutput, 0.5f);
			gearRackFrontTwo.motor.set(ControlMode.PercentOutput, 0.5f);
			gearRackBackOne.motor.set(ControlMode.PercentOutput, 0.5f);
			gearRackBackTwo.motor.set(ControlMode.PercentOutput, 0.5f);
		} else {
			gearRackFrontOne.motor.set(ControlMode.PercentOutput, 0.0f);
			gearRackFrontTwo.motor.set(ControlMode.PercentOutput, 0.0f);
			gearRackBackOne.motor.set(ControlMode.PercentOutput, 0.0f);
			gearRackBackTwo.motor.set(ControlMode.PercentOutput, 0.0f);
		}

		if (driver.getRawButton(8)) {
			gearRackBackTwo.ResetEncoder();
			gearRackBackOne.ResetEncoder();
		}
	}

	public void UpdateMotors() {
		driveTrain.Update();
	}

	public float TranslateController(float input) {
		float deadzone = 0.15f;
		if (input > -deadzone && input < deadzone) {
			input = 0.0f;
		}
		float a = 0.7f;
		float output = (a * input * input * input) + (1 - a) * input;
		return output;
	}

	public void ControllerDrive() {
		float horJoystick = TranslateController((float) driver.getRawAxis(0));
		float verJoystick = TranslateController((float) driver.getRawAxis(5));

		driveTrain.SetRightSpeed(-verJoystick + -horJoystick);
		driveTrain.SetLeftSpeed(-verJoystick + horJoystick);
	}

	public void Intake(float speeed) {
		intake.set(ControlMode.PercentOutput, -speeed);
	}

	public void Outtake(float speeed) {
		intake.set(ControlMode.PercentOutput, speeed);
	}

	public void HatchRelease() {
		hatch.set(DoubleSolenoid.Value.kReverse);
		hatchReleased = true;
	}

	public void HatchHold() {
		hatch.set(DoubleSolenoid.Value.kForward);
		hatchReleased = false;
	}

	public void ArmMove(float speeed) {
		armLeft.set(ControlMode.PercentOutput, speeed);
		armRight.set(ControlMode.PercentOutput, -speeed);
	}

	public void Climb() {

	}

	public void DrivetrainBrakes(boolean brakes) {
		if (brakes = true) {
			// driveTrain.SetBreak();
		} else {
			// driveTrain.SetCoast();
		}
	}

	public void UpdateRampTarget(float newTarget) {
		if (newTarget != rampTargetPoint) {
			rampTargetPoint = newTarget;
			rampStep = Math.abs(rampCurrent - rampTargetPoint) / (rampLengthSeconds / timerDelaySeconds);
		}
	}
}
