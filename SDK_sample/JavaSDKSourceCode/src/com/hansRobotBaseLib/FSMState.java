package com.hansRobotBaseLib;

public enum FSMState {
	UnInitialize(0),
	Initialize(1),
	ElectricBoxDisconnect(2),
	ElectricBoxConnecting(3),
	EmergencyStopHandling(4),
	EmergencyStop(5),
	Blackouting48V(6),
	Blackout_48V(7),
	Electrifying48V(8),
	SafeGuardErrorHandling(9),
	SafeGuardError(10),
	SafeGuardHandling(11),
	SafeGuard(12),
	ControllerDisconnecting(13),
	ControllerDisconnect(14),
	ControllerConnecting(15),
	ControllerVersionError(16),
	EtherCATError(17),
	ControllerChecking(18),
	Reseting(19),
	RobotOutofSafeSpace(20),
	RobotCollisionStop(21),
	Error(22),
	RobotEnabling(23),
	Disable(24),
	Moving(25),
	LongJogMoving(26),
	RobotStopping(27),
	RobotDisabling(28),
	RobotOpeningFreeDriver(29),
	RobotClosingFreeDriver(30),
	FreeDriver(31),
	RobotHolding(32),
	StandBy(33),
	ScriptRunning(34),
	ScriptHoldHandling(35),
	ScriptHolding(36),
	ScriptStopping(37),
	ScriptStopped(38),
	HRAppDisconnected(39),
	HRAppError(40),
	State_Count(41);

	
	private int value = 0;
	
	private FSMState(int i)
	{
		this.value = i;
	}
	
	public static String valueof(int value)
	{
		switch(value)
		{
		case 0:
			return "UnInitialize";
		case 1:
			return "Initialize";
		case 2:
			return "ElectricBoxDisconnect";
		case 3:
			return "ElectricBoxConnecting";
		case 4:
			return "EmergencyStopHandling";
		case 5:
			return "EmergencyStop";
		case 6:
			return "Blackouting48V";
		case 7:
			return "Blackout_48V";
		case 8:
			return "Electrifying48V";
		case 9:
			return "SafeguardErrorHandling";
		case 10:
			return "SafeguardError";
		case 11:
			return "SafeguardHandling";
		case 12:
			return "Safeguarding";
		case 13:
			return "ControllerDisconnecting";
		case 14:
			return "ControllerDisconnect";
		case 15:
			return "ControllerConnecting";
		case 16:
			return "ControllerVersionError";
		case 17:
			return "EtherCATError";
		case 18:
			return "ControllerChecking";
		case 19:
			return "Reseting";
		case 20:
			return "RobotOutofSafeSpace";
		case 21:
			return "RobotCollisionStop";
		case 22:
			return "Error";
		case 23:
			return "RobotEnabling";
		case 24:
			return "Disable";
		case 25:
			return "Moving";
		case 26:
			return "LongJogMoving";
		case 27:
			return "RobotStopping";
		case 28:
			return "RobotDisabling";
		case 29:
			return "RobotOpeningFreeDriver";
		case 30:
			return "RobotClosingFreeDriver";
		case 31:
			return "FreeDriver";
		case 32:
			return "RobotHolding";
		case 33:
			return "StandBy";
		case 34:
			return "ScriptRunning";
		case 35:
			return "ScriptHoldHandling";
		case 36:
			return "ScriptHolding";
		case 37:
			return "ScriptStopping";
		case 38:
			return "ScriptStopped";
		case 39:
			return "HRAppDisconnected";
		case 40:
			return "HRAppError";
		case 41:
			return "RobotLoadIdentify";
		case 42:
			return "Braking";
		default:
			return null;
		}
	}
	
	public int value()
	{
		return this.value;
	}
}

