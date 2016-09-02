package org.team3128.main;

import org.team3128.autonomous.programs.TestGyroTurnAuto;
import org.team3128.common.NarwhalRobot;
import org.team3128.common.drive.TankDrive;
import org.team3128.common.hardware.encoder.velocity.QuadratureEncoderLink;
import org.team3128.common.hardware.lights.LightsColor;
import org.team3128.common.hardware.lights.LightsSequence;
import org.team3128.common.hardware.lights.PWMLights;
import org.team3128.common.hardware.motor.MotorGroup;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerXbox;
import org.team3128.common.util.GenericSendableChooser;
import org.team3128.common.util.Log;
import org.team3128.common.util.units.Length;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Main class for our 2015 robot, The Clawwww minus the Claw (www) ( just drive ).
 * @author Jamie (modified by Wesley)
 *
 */
public class MainDriveCold extends NarwhalRobot
{
	
	public ListenerManager lmXbox;
	
	public MotorGroup _pidTestMotor;
	
	public MotorGroup leftMotors;
	public MotorGroup rightMotors;
	public QuadratureEncoderLink leftDriveEncoder;
	public QuadratureEncoderLink rightDriveEncoder;
	public PowerDistributionPanel powerDistPanel;
	
	public TankDrive drive;
	
	PWMLights lights;
	
	LightsSequence lightShowSequence;
	
	AnalogGyro gyro;
	
	@Override
	protected void updateDashboard()
	{
		SmartDashboard.putNumber("Total Current: ", powerDistPanel.getTotalCurrent());
		
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
	}

	@Override
	protected void constructHardware()
	{
		lmXbox = new ListenerManager(new Joystick(0));
		powerDistPanel = new PowerDistributionPanel();
		
		leftDriveEncoder = new QuadratureEncoderLink(0,	1, 128, false);
		rightDriveEncoder = new QuadratureEncoderLink(3, 4, 128, true);
		
		leftMotors = new MotorGroup();
		leftMotors.addMotor(new Talon(0));
		leftMotors.addMotor(new Talon(1));
		
		
		rightMotors = new MotorGroup();
		rightMotors.addMotor(new Talon(2));
		rightMotors.addMotor(new Talon(3));
		rightMotors.invert();
	
		//TODO: remeasure wheelbase and track
		drive = new TankDrive(leftMotors, rightMotors, leftDriveEncoder, rightDriveEncoder,
				5.75 * Length.in * Math.PI, 1, 27 * Length.in, 15.125 * Length.in);
		

				
		lights = new PWMLights(10, 11, 12);
		
		gyro = new AnalogGyro(0);
		gyro.setSensitivity(-0.007); //invert gyro
		
		lightShowSequence = new LightsSequence();

		lightShowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(0xff, 1, 1), 500, false));
		lightShowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(0xff, 0xff, 1), 500, false));
		lightShowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(1, 0xff, 1), 500, false));
		lightShowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(1, 0xff, 0xff), 500, false));
		lightShowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(1, 1, 0xff), 500, false));
		lightShowSequence.addStep(new LightsSequence.Step(LightsColor.new8Bit(0xff, 1, 0xff), 500, false));
		
		lightShowSequence.setRepeat(true);
		
		LightsSequence ccaColorsSequence = new LightsSequence();
		ccaColorsSequence.addStep(new LightsSequence.Step(LightsColor.white, 1000, false));
		ccaColorsSequence.addStep(new LightsSequence.Step(LightsColor.red, 1000, false));
		ccaColorsSequence.setRepeat(true);
		
		lights.executeSequence(ccaColorsSequence);	
		
		addListenerManager(lmXbox);
		
        Log.info("MainDriveCold", "Coldbot   Activated");
	}

	@Override
	protected void setupListeners()
	{
		//Teleop listeners
		//--------------------------------------------------------------------------------------------

        //NIVision.IMAQdxStartAcquisition(cameraHandle);
		
		//-----------------------------------------------------------
		// Drive code, on Logitech Extreme3D joystick
		//-----------------------------------------------------------
		lmXbox.nameControl(ControllerXbox.START, "ClearStickyFaults");
		lmXbox.nameControl(ControllerXbox.BACK, "ResetGyro");

		lmXbox.nameControl(ControllerXbox.RB, "DriveDoubleSpeed");
		
		lmXbox.nameControl(ControllerXbox.JOY2X, "DriveTurn");
		lmXbox.nameControl(ControllerXbox.JOY1Y, "DriveForwardBackward");
		
		lmXbox.addButtonDownListener("ClearStickyFaults", () ->
		{
			powerDistPanel.clearStickyFaults();
		});
		
		lmXbox.addButtonDownListener("ResetGyro", () ->
		{
			gyro.reset();
		});
		
		
		lmXbox.addMultiListener(() ->
		{
			double joyX = .75 * lmXbox.getAxis("DriveTurn");
			double joyY = lmXbox.getAxis("DriveForwardBackward");			
			drive.arcadeDrive(joyX, joyY, 1, lmXbox.getButton("DriveDoubleSpeed"));
		}, "DriveTurn", "DriveForwardBackward", "DriveDoubleSpeed");		
	}

	@Override
	protected void teleopInit()
	{
		
	}

	@Override
	protected void autonomousInit()
	{
		
	}
	
	@Override
    protected void constructAutoPrograms(GenericSendableChooser<CommandGroup> programChooser) 
    {
    	programChooser.addDefault("Test Gyro Turn", new TestGyroTurnAuto(drive, gyro));
    }
}
