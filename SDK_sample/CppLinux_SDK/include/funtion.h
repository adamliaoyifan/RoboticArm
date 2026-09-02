#ifndef funtion_h
#define funtion_h

void UserCase_XToStandby();
void UserCase_SetSimRobot();
void UserCase_BrakeControl();
void UserCase_Grap();
void UserCase_Script();
void UserCase_ControlBoxIO();
void UserCase_BoxIDSample(string strIP1,string strIP2);
void UserCase_SysStateShow();
void UserCase_PonitInfo();
void UserCase_RbtStateShow();
void UserCase_CoordTran();
void UserCase_ConfigTcpUcs();
void UserCase_ForceControl();
void UserCase_FreeDrive();
void UserCase_MoveCircle();
void UserCase_MoveInStandby();
void UserCase_WayPoint();
void UserCase_MoveInStartPoint();
void UserCase_MovePath();
void UserCase_MovepathJ();
void UserCase_MovePathL();
void UserCase_ServoJ();
void UserCase_ServoP();
void UserCase_Trace();
void UserCase_ConnectToModBus();
void ParsePortJsondataDemo(const string &strIP);
void ParsePortJson(const string &strIP);
void UserCase_CalibrationForce();
#endif 