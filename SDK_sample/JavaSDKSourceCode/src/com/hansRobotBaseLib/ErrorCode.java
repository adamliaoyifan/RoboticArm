package com.hansRobotBaseLib;

public enum ErrorCode {
	REC_Succeed(0),isNotConnect(39500), paramsError(39501),returnError(39502),
	StateRefuse(20018), SocketError(39503), Connect2CPSFailed(39504), CmdError(39505);

	
	private int value = 0;
	
	private ErrorCode(int value)
	{
		this.value = value;
	}

	public static ErrorCode valueof(int value)
	{
		switch(value)
		{
		case 0:
			return REC_Succeed;
		case 39500:
			return isNotConnect;
		case 39501:
			return paramsError;
		case 20018:
			return StateRefuse;
		case 39502:
			return returnError;
		case 39503:
			return SocketError;
		case 39504:
			return Connect2CPSFailed;
		case 39505:
			return CmdError;
		default:
			return null;
		}
	}
	
	public int value()
	{
		return this.value;
	}
}

