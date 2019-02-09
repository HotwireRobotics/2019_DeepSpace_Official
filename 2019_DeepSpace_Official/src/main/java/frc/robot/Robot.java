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
	public DoubleSolenoid diskBrake = new DoubleSolenoid(2, 3);
	public boolean buttonReleased;
	public boolean hatchReleased;

	// Joysticks
	public Joystick driver;
	public Joystick operator;
	public Joystick debug;

	// Arm
	public TalonSRX armLeft = new TalonSRX(3);
	public TalonSRX armRight = new TalonSRX(4);
	public boolean armHold = true;
	public boolean povReleased = false;
	public double potTarget = 0;
	public double currentBuffer;
	public double groundTarget = 0.8678;
	public double shipTarget = 0.67;
	public double rocketTarget = 0.78;
	public double hatchTarget = 0.84;
	public float downForce = 0.0f;
	//TODO

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

		potTarget = pot.get();

		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);
	}

	public void teleopPeriodic() {

		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");

		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);

		// Driver Controls

		// Limelight
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


			// Intake
			if (operator.getRawButton(1)) {
				Intake(0.9f);
			} else {
				if (operator.getRawButton(4)) {
					Outtake(0.4f);
				} else {
					Outtake(0.0f);
				}
			}

			// Arm Targets
			if (povReleased == true && operator.getPOV() != -1) {
				povReleased = false;
				armHold = false;
				DiskBrakeDisable();

				currentBuffer = 0.005;
				if (operator.getPOV() == 0) {

				} else if (operator.getPOV() == 180) {
					potTarget = groundTarget;
					downForce = 0.1f;
				} else if (operator.getPOV() == 90) {
					potTarget = shipTarget;
					downForce = 0.0f;
				} else if(operator.getPOV() == 270){
					potTarget = hatchTarget;
					downForce = 0.075f;
				}
			}
			if (operator.getPOV() == -1) {
				povReleased = true;
			}

			if (pot.get() > potTarget + currentBuffer) {
				DiskBrakeDisable();
				ArmMove(0.45f);
				System.out.println("up");
			} else if (pot.get() < potTarget - currentBuffer) {
				DiskBrakeDisable();
				
				ArmMove(downForce);
				System.out.println("down");

			} else {
				DiskBrakeEnable();
				ArmMove(0);
				System.out.println("stop");
				float bufferGrowth = 1.2f;
				currentBuffer = currentBuffer * bufferGrowth;
			}
			

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
		}

		UpdateMotors();
	}

	public void testInit() {
		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);
		DiskBrakeDisable();
	}

	public void testPeriodic() {

		System.out.println(pot.get());
		ControllerDrive();
		UpdateMotors();
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

	public void DiskBrakeEnable() {
		diskBrake.set(DoubleSolenoid.Value.kForward);

	}

	public void DiskBrakeDisable() {
		diskBrake.set(DoubleSolenoid.Value.kReverse);

	}
}