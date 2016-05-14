package org.team3128.main;

import org.team3128.autonomous.programs.DriveIntoAutoZoneAuto;
import org.team3128.autonomous.programs.DualFarCanGrabAuto;
import org.team3128.autonomous.programs.FarCanGrabAuto;
import org.team3128.autonomous.programs.TakeToteIntoZoneAuto;
import org.team3128.common.autonomous.DoNothingAuto;
import org.team3128.common.drive.TankDrive;
import org.team3128.common.hardware.encoder.angular.AnalogPotentiometerEncoder;
import org.team3128.common.hardware.encoder.angular.IAngularEncoder;
import org.team3128.common.hardware.encoder.velocity.QuadratureEncoderLink;
import org.team3128.common.hardware.lights.PWMLights;
import org.team3128.common.hardware.motor.MotorGroup;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerAttackJoy;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.multibot.MainClass;
import org.team3128.common.multibot.RobotTemplate;
import org.team3128.common.util.GenericSendableChooser;
import org.team3128.common.util.Log;
import org.team3128.common.util.units.Length;
import org.team3128.mechanisms.ClawArm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Main class for our 2015 robot, The Clawwww.
 * @author Jamie
 *
 */
public class MainTheClawwww extends MainClass
{
	
	/**
	 * Multiplier for teleop arm speed
	 */
	public static double armSpeedMultiplier = .8;
	
	public ListenerManager lmExtreme;
	public ListenerManager lmJoyLeft;
	public ListenerManager lmJoyRight;
	
	public MotorGroup _pidTestMotor;
	
	public MotorGroup leftMotors;
	public MotorGroup rightMotors;
	public QuadratureEncoderLink leftDriveEncoder;
	public QuadratureEncoderLink rightDriveEncoder;
	
	public MotorGroup armTurnMotor;
	
	public MotorGroup armJointMotor;
	
	public MotorGroup frontHookMotor;
	
	public MotorGroup clawGrabMotor;

	public IAngularEncoder armRotateEncoder;
	
	public AnalogPotentiometerEncoder armJointEncoder;
	
	public PowerDistributionPanel powerDistPanel;
	
	public TankDrive drive;
	
	public ClawArm clawArm;
		
	boolean codDriveEnabled = false;
	boolean shoulderInverted = true;
	boolean elbowInverted = true;
	
	PWMLights lights;
	
	public MainTheClawwww()
	{	
		lmExtreme = new ListenerManager(new Joystick(0));
		lmJoyLeft = new ListenerManager(new Joystick(2));
		lmJoyRight = new ListenerManager(new Joystick(1));		
		powerDistPanel = new PowerDistributionPanel();
		
		leftDriveEncoder = new QuadratureEncoderLink(0,	1, 128, false);
		rightDriveEncoder = new QuadratureEncoderLink(3, 4, 128, true);
		
		leftMotors = new MotorGroup(/*new PIDSpeedTarget(0, leftDriveEncoder, new VelocityPID(.1, 0, 0))*/);
		leftMotors.addMotor(new Talon(1));
		leftMotors.addMotor(new Talon(2));
		//leftMotors.startControl(0);
		
		
		rightMotors = new MotorGroup(/*new PIDSpeedTarget(0, rightDriveEncoder, new VelocityPID(.1, 0, 0))*/);
		rightMotors.addMotor(new Talon(3));
		rightMotors.addMotor(new Talon(4));
		rightMotors.invert();
		//rightMotors.startControl(0);
		
		armTurnMotor = new MotorGroup();
		armTurnMotor.addMotor(new Talon(6));
		armTurnMotor.invert();
		

		armRotateEncoder = new AnalogPotentiometerEncoder(0, 0, 4.829, 300);
		
		armJointMotor = new MotorGroup();
		armJointMotor.addMotor(new Talon(5));
		
		armJointEncoder = new AnalogPotentiometerEncoder(1, 0, 4.829, 300);
		
		frontHookMotor = new MotorGroup();
		frontHookMotor.addMotor(new Talon(9));
		
		clawGrabMotor = new MotorGroup();
		clawGrabMotor.addMotor(new Talon(8));
		
		//lights = new PWMLights(0, 9, 7);

		clawArm = new ClawArm(armTurnMotor, armJointMotor, clawGrabMotor, armRotateEncoder, armJointEncoder, powerDistPanel);

		drive = new TankDrive(leftMotors, rightMotors, leftDriveEncoder, rightDriveEncoder, 6 * Length.in * Math.PI, 1, 24.5 * Length.in);
		
		
		lights = new PWMLights(10, 11, 12);

	}

	protected void initializeRobot(RobotTemplate robotTemplate)
	{	
		robotTemplate.addListenerManager(lmExtreme);
		robotTemplate.addListenerManager(lmJoyLeft);
		robotTemplate.addListenerManager(lmJoyRight);
		
        Log.info("MainTheClawwww", "\"The Clawwwwwww.....\"   Activated");
        
        //Teleop Listeners
        //---------------------------------------------------------------------------------------
        
		lmExtreme.nameControl(new Button(12), "ClearStickyFaults");
		lmExtreme.nameControl(ControllerExtreme3D.TRIGGER, "DriveDoubleSpeed");
		
		lmExtreme.nameControl(ControllerExtreme3D.TWIST, "DriveTurn");
		lmExtreme.nameControl(ControllerExtreme3D.JOYY, "DriveForwardBackward");
		lmExtreme.nameControl(ControllerExtreme3D.JOYY, "DriveThrottle");
		
		lmJoyRight.nameControl(ControllerAttackJoy.JOYY, "RotateShoulder");
		
		lmJoyLeft.nameControl(ControllerAttackJoy.JOYY, "RotateElbow");

		
		//-----------------------------------------------------------
		// Drive code, on Logitech Extreme3D joystick
		//-----------------------------------------------------------
		
		lmExtreme.addButtonDownListener("ClearStickyFaults", () ->
		{
			powerDistPanel.clearStickyFaults();
		});
		
		
		lmExtreme.addMultiListener(() ->
		{
			double joyX = lmExtreme.getAxis("DriveTurn");
			double joyY = lmExtreme.getAxis("DriveForwardBackward");			
			double throttle = -lmExtreme.getAxis("DriveThrottle");

			drive.arcadeDrive(joyX, joyY, throttle, lmExtreme.getButton("DriveDoubleSpeed"));
			
		}, "DriveTurn", "DriveForwardBackward", "DriveDoubleSpeed");
		
		//-----------------------------------------------------------
		// Arm control code, on joysticks
		//-----------------------------------------------------------
		
		lmJoyRight.addListener("RotateShoulder", (double value) ->
		{
			double power = (shoulderInverted ? armSpeedMultiplier : -armSpeedMultiplier) * value;
			
			if(power > 0)
			{
				power /= 1.5;
			}			
			
			clawArm.onArmJoyInput(power);
			
			
		});
		
		lmJoyLeft.addListener("RotateElbow", (double value) ->
		{
			clawArm.onJointJoyInput((elbowInverted ? armSpeedMultiplier : -armSpeedMultiplier) * value);
		});
		
//		lmJoyRight.addButtonDownListener(new Button(2), () -> shoulderInverted = false);
//		lmJoyRight.addButtonDownListener(new Button(3), () -> shoulderInverted = true);
//		lmJoyRight.addButtonDownListener(new Button(6), () -> shoulderInverted = true);
//		lmJoyRight.addButtonDownListener(new Button(7), () -> shoulderInverted = false);
//		lmJoyLeft.addButtonDownListener(new Button(2), () -> elbowInverted = false);
//		lmJoyLeft.addButtonDownListener(new Button(3), () -> elbowInverted = true);
//		lmJoyLeft.addButtonDownListener(new Button(6), () -> elbowInverted = true);
//		lmJoyLeft.addButtonDownListener(new Button(7), () -> elbowInverted = false);
//		
//		lmJoyRight.addListener(new Button(1), () -> clawGrabMotor.setTarget(0.8));
//		lmJoyRight.addListener(ControllerAttackJoy.UP1, () -> clawGrabMotor.setTarget(0));
//		lmJoyLeft.addListener(new Button(1), () -> clawGrabMotor.setTarget(-0.8));
//		lmJoyLeft.addListener(ControllerAttackJoy.UP1, () -> clawGrabMotor.setTarget(0));
//		
//		lmJoyRight.addListener(new Button(4), () -> clawGrabMotor.setTarget(0.8));
//		lmJoyRight.addListener(ControllerAttackJoy.UP4, () -> clawGrabMotor.setTarget(0));
//		lmJoyRight.addListener(new Button(5), () -> clawGrabMotor.setTarget(-0.8));
//		lmJoyRight.addListener(ControllerAttackJoy.UP5, () -> clawGrabMotor.setTarget(0));
//		lmExtreme.addListener(ControllerExtreme3D.DOWN3, () -> frontHookMotor.setTarget(0.3));
//		lmExtreme.addListener(ControllerExtreme3D.UP3, () -> frontHookMotor.setTarget(0));
//		lmExtreme.addListener(ControllerExtreme3D.DOWN4, () -> frontHookMotor.setTarget(-0.3));
//		lmExtreme.addListener(ControllerExtreme3D.UP4, () -> frontHookMotor.setTarget(0));
//		lmExtreme.addListener(ControllerExtreme3D.DOWN5, () -> frontHookMotor.setTarget(0.3));
//		lmExtreme.addListener(ControllerExtreme3D.UP5, () -> frontHookMotor.setTarget(0));
//		lmExtreme.addListener(ControllerExtreme3D.DOWN6, () -> frontHookMotor.setTarget(-0.3));
//		lmExtreme.addListener(ControllerExtreme3D.UP6, () -> frontHookMotor.setTarget(0));
//
//		lmExtreme.addListener(ControllerExtreme3D.UP8, () -> frontHookMotor.setTarget(0));

//		lmExtreme.addListener(Always.instance, () -> {
//			int red = RobotMath.clampInt(RobotMath.floor_double_int(255 * (powerDistPanel.getTotalCurrent() / 30.0)), 0, 255);
//			int green = 255 - red;
//			
//			LightsColor color = LightsColor.new8Bit(red, green, 0);
//			lights.setColor(color);
//			
//			//Log.debug("ArmAngle", armRotateEncoder.getAngle() + " degrees");
//		});
		
	}

	protected void initializeDisabled()
	{
		
		armTurnMotor.resetSpeedControl();
		armJointMotor.resetSpeedControl();
		clawArm.switchJointToManualControl();
		
		clawArm.stopClawLimitThread();
				
		leftMotors.resetSpeedControl();
		rightMotors.resetSpeedControl();
		
		clawArm.resetTargets();
	}

	protected void initializeAuto()
	{
		//lights.setColor(Color.new4Bit(0xa, 2, 2));
		
		//reset PID error
		armTurnMotor.resetSpeedControl();
		clawArm.resetTargets();
	}
	
	protected void initializeTeleop()
	{	
		clawArm.resetTargets();

		//lights.setFader(Color.new11Bit(2000, 2000, 2000), 1, 10);
		

					

		
		//clawArm.startClawLimitThread();
	}

	@Override
	protected void addAutoPrograms(GenericSendableChooser<CommandGroup> autoChooser)
	{
		autoChooser.addDefault("Take Tote into Auto Zone", new TakeToteIntoZoneAuto(drive, frontHookMotor, lights));
		autoChooser.addObject("Can Grab", new FarCanGrabAuto(drive, clawArm, frontHookMotor, false));
		autoChooser.addObject("Can Grab w/ Tote Pickup", new FarCanGrabAuto(drive, clawArm, frontHookMotor, true));
		autoChooser.addObject("Dual Can Grab", new DualFarCanGrabAuto(drive, clawArm));
		autoChooser.addObject("Drive Into Auto Zone", new DriveIntoAutoZoneAuto(drive, lights));
		autoChooser.addObject("Do Nothing", new DoNothingAuto(lights));
	}

	@Override
	protected void updateDashboard()
	{
		SmartDashboard.putNumber("armRotateEncoder", armRotateEncoder.getAngle());
		SmartDashboard.putNumber("Total Current: ", powerDistPanel.getTotalCurrent());
	}
}
