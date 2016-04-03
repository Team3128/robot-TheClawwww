package org.team3128.autonomous.programs;

import org.team3128.autonomous.commands.CmdToteGrab;
import org.team3128.common.drive.TankDrive;
import org.team3128.common.hardware.lights.LightsColor;
import org.team3128.common.hardware.lights.PWMLights;
import org.team3128.common.hardware.motor.MotorGroup;
import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TakeToteIntoZoneAuto extends CommandGroup
{
    public TakeToteIntoZoneAuto(TankDrive tankDrive, MotorGroup toteHookMotor, PWMLights lights)
    {
    	addSequential(tankDrive.new CmdMoveForward(2 * Length.cm, 1000, true));
    	addSequential(new CmdToteGrab(toteHookMotor, 700));
    	addSequential(tankDrive.new CmdInPlaceTurn(90, 4000, Direction.RIGHT));
    	addSequential(tankDrive.new CmdMoveForward(200 * Length.cm, 0, true));
    	
    	lights.setColor(LightsColor.new8Bit(1, 1, 0xff));
    }
}
