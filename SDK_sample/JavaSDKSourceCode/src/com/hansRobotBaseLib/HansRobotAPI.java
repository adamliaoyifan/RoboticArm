package com.hansRobotBaseLib;

import java.util.Vector;

public class HansRobotAPI extends VariableType{
	public static final int MaxBox = 5;
	HansRobotAPI_Base[] m_client =new HansRobotAPI_Base[MaxBox] ;
	private Thread monitorThread=null;



	private void startMonitorThreadIfNeeded()
	{
		if (monitorThread == null)
		{
			monitorThread = new Thread(() -> {monitorConnectionStatus();});
			monitorThread.setDaemon(true);
			monitorThread.start();
		}
	}

	private void monitorConnectionStatus() {

		boolean shouldContinue = true;  // 控制循环是否继续
		int nRet;
		IntData Ret = new IntData();

		// 不断检查是否接收到新数据
		while (shouldContinue) {
			try {
				for(int boxID = 0;boxID<MaxBox;boxID++)
				{
					if(m_client[boxID]!=null)
					{
						long currentTime = System.currentTimeMillis();
						long lastCallTIme = m_client[boxID].GetLastCallTime();  // 记录最后接收到数据的时间
						if (currentTime - lastCallTIme > 5 * 1000) {
							// 超过 20 秒未接收到数据，进行超时处理
							nRet = m_client[boxID].HRIF_IsSimulateRobot_base(Ret);
							if(nRet==ErrorCode.SocketError.value()) {
								handleTimeout(boxID);
								shouldContinue = false;
							}// 停止循环
						}
						// 每隔 100 毫秒检查一次
						Thread.sleep(5000);
					}
				}

			} catch (InterruptedException e) {
				e.printStackTrace();
				shouldContinue = false;  // 出现异常时停止循环
			}
		}
	}

	// 连接丢失时的处理逻辑
	private void handleTimeout(int boxID) {
		m_client[boxID].HRIF_DisConnect_base();
		// 这里可以加入你自己的逻辑来处理连接丢失后的情况
		System.err.println("连接丢失");
		// 可以抛出异常或进行其他处理
		throw new RuntimeException("Socket连接断开 ");
	}


	public void SetAutoTestState(int boxID,boolean state)
	{
		m_client[boxID].SetAutoTestState(state);
	}
	
	/**
	 * @Title HRIF_Connect 
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param hostName 控制器IP地址
	 * @param nPort 端口号
	 * @return int,0:调用成功;>0返回错误码
	 * @Description 连接机器人服务器
	 */

	public int HRIF_Connect(int boxID, String hostName, int nPort) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
//		api_class[boxID] = new HansRobotAPI();
		if (m_client[boxID] == null)
		{
			m_client[boxID] = new HansRobotAPI_Base();
		}

		int nRet = m_client[boxID].HRIF_Connect_base(hostName, nPort);
		if(nRet!=0)
		{
			m_client[boxID].HRIF_DisConnect_base();
			return ErrorCode.Connect2CPSFailed.value();
		}
		startMonitorThreadIfNeeded();
		return nRet;
	}
	
	/**
	 * @Title HRIF_DisConnect 
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 断开连接机器人服务器
	 */
	public int HRIF_DisConnect(int boxID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_DisConnect_base();
	}
	
	/**
	 * @Title HRIF_IsConnected 
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @return boolean,true 已连接，false 未连接
	 * @Description 判断控制器是否连接
	 */
	public boolean  HRIF_IsConnected(int boxID) {
		if(boxID >= MaxBox) {
			return false;
		}
		return m_client[boxID].HRIF_IsConnected_base();
	}
	
	/**
	 * @Title HRIF_ShutdownRobot 
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 控制器断电 (断开机器人供电，系统关机 )
	 */
	public int HRIF_ShutdownRobot(int boxID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ShutdownRobot_base();
	}
	
	/**
	 * @Title HRIF_Connect2Box
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 连接控制器电箱
	 */
	public int HRIF_Connect2Box(int boxID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_Connect2Box_base();
	}

	/**
	 * @Title HRIF_Electrify
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 机器人上电
	 */
	public int HRIF_Electrify(int boxID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_Electrify_base();
	}

	/**
	 * @Title HRIF_Blackout
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 机器人断电
	 */
	public int HRIF_Blackout(int boxID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_Blackout_base();
	}

	/**
	 * @Title HRIF_Connect2Controller
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 连接控制器，连接过程中会启动主站，初始化从站，配置参数，检查配置，完成后跳转到去
使能状态
	 */
	public int HRIF_Connect2Controller(int boxID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_Connect2Controller_base();
	}

	/**
	 * @Title HRIF_IsSimulateRobot
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param nSimulateRobot 是否模拟机器人
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description  获取是否为模拟机器人
	 */
	public int HRIF_IsSimulateRobot(int boxID, IntData nSimulateRobot) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_IsSimulateRobot_base( nSimulateRobot);
	}

	/**
	 * @Title HRIF_IsControllerStarted
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param nStarted 是否启动完成
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description  控制器是否启动完成
	 */
	public int HRIF_IsControllerStarted(int boxID, IntData nStarted ) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_IsControllerStarted_base(nStarted);
	}

	/**
	 * @Title HRIF_SetSimulation
	 * @date 2024年9月23日
	 * @param boxID 电箱ID
	 * @param nSimulation
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description  设置机器人的模拟状态
	 */
	public int HRIF_SetSimulation(int boxID, int Robot,int nSimulation ) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetSimulation_base(Robot, nSimulation);
	}

	/**
	 * @Title HRIF_GetErrorCodeStr
	 * @param boxID 电箱ID
	 * @param nErrorCode 错误码
	 * @param strErrorMsg 错误码描述
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description  获取错误码解释
	 */
	public int HRIF_GetErrorCodeStr(int boxID, int nErrorCode, StringData strErrorMsg) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GetErrorCodeStr_base(nErrorCode,strErrorMsg);
	}

	/**
	 * @Title HRIF_ReadVersion
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param strVer 整体版本号
	 * @param nCPSVersion CPS版本
	 * @param nCodesysVersion 控制器版本
	 * @param nBoxVerMajor 电箱版本
	 * @param nBoxVerMid 控制板固件版本
	 * @param nBoxVerMin 控制板固件版本
	 * @param nAlgorithmVer 算法版本
	 * @param nElfinFirmwareVer 固件版本
	 * @param sSoftwareVersion 软件版本号
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取控制器版本号
	 */
	public int HRIF_ReadVersion(int boxID, int rbtID, StringData strVer, IntData nCPSVersion, IntData nCodesysVersion, IntData nBoxVerMajor,
			IntData nBoxVerMid, IntData nBoxVerMin,IntData nAlgorithmVer,IntData nElfinFirmwareVer, StringData sSoftwareVersion)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadVersion_base(rbtID,strVer,nCPSVersion,nCodesysVersion,nBoxVerMajor,nBoxVerMid,nBoxVerMin,nAlgorithmVer,nElfinFirmwareVer,sSoftwareVersion);
	}

	/**
	 * @Title HRIF_ReadRobotModel
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param strModel 机器人类型
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取机器人类型
	 */
	public int HRIF_ReadRobotModel(int boxID, StringData strModel) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadRobotModel_base( strModel);
	}
	
	/**-----------------------Part2 轴组控制-------------------------*/
	/**
	 * @Title HRIF_GrpEnable 
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 机器人使能命令
	 */
	public int HRIF_GrpEnable(int boxID, int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GrpEnable_base(rbtID);
	}
	
	/**
	 * @Title HRIF_GrpDisable
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 机器人去使能命令
	 */
	public int HRIF_GrpDisable(int boxID, int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GrpDisable_base(rbtID);
	}

	/**
	 * @Title HRIF_GrpReset
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 机器人复位命令
	 */
	public int HRIF_GrpReset(int boxID, int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GrpReset_base(rbtID);
	}

	/**
	 * @Title HRIF_GrpStop
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 机器人停止运动命令
	 */
	public int HRIF_GrpStop(int boxID, int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GrpStop_base(rbtID);
	}

	/**
	 * @Title HRIF_GrpInterrupt
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 机器人暂停运动命令
	 */
	public int HRIF_GrpInterrupt(int boxID, int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GrpInterrupt_base(rbtID);
	}

	/**
	 * @Title HRIF_GrpContinue
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 机器人继续运动命令，机器人处于暂停状态时有效
	 */
	public int HRIF_GrpContinue(int boxID, int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GrpContinue_base(rbtID);
	}

	/**
	 * @Title HRIF_GrpCloseFreeDriver
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 关闭自由驱动
	 */
	public int HRIF_GrpCloseFreeDriver(int boxID, int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GrpCloseFreeDriver_base(rbtID);
	}

	/**
	 * @Title HRIF_GrpOpenFreeDriver
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 打开自由驱动
	 */
	public int HRIF_GrpOpenFreeDriver(int boxID, int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GrpOpenFreeDriver_base(rbtID);
	}

	/**
	 * @Title HRIF_EnterSafetyGuard
	 * @date 2024年9月11日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param bFlag 状态标识
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 强制进入安全光幕（软急停）
	 */
	public int HRIF_EnterSafeGuard(int boxID,int rbtID, int bFlag) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_EnterSafeGuard_base(rbtID, bFlag);
	}

	/**
	 * @Title HRIF_OpenBrake
	 * @date 2024年9月9日
	 * @param boxID 电箱ID
	 * @param nAxisID 轴ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 松闸
	 */
	public int HRIF_OpenBrake(int boxID,int rbtID, int nAxisID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_OpenBrake_base(rbtID, nAxisID);
	}

	/**
	 * @Title HRIF_CloseBrake
	 * @date 2024年9月9日
	 * @param boxID 电箱ID
	 * @param nAxisID 轴ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 抱闸
	 */
	public int HRIF_CloseBrake(int boxID,int rbtID, int nAxisID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_CloseBrake_base(rbtID, nAxisID);
	}

	/**
	 * @Title HRIF_ReadBrakeStatus
	 * @param rbtID   机器人ID
	 * @param stateJ1 松/抱闸状态
	 * @param stateJ2
	 * @param stateJ3
	 * @param stateJ4
	 * @param stateJ5
	 * @param stateJ6
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2024年9月9日
	 * @Description 抱闸
	 */
	public int HRIF_ReadBrakeStatus(int boxID,int rbtID, IntData stateJ1,IntData stateJ2,IntData stateJ3,IntData stateJ4,IntData stateJ5,IntData stateJ6) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadBrakeStatus_base(rbtID,stateJ1,stateJ2,stateJ3,stateJ4,stateJ5,stateJ6);
	}

	/**
	 * @Title HRIF_MoveToSS
	 * @param rbtID   机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2024年9月9日
	 * @Description 移动至安全位置
	 */
	public int HRIF_MoveToSS(int boxID,int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveToSS_base(rbtID);
	}

	/**
	 * @Title HRIF_ReadTriStageSwitch
	 * @param rbtID   机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2024年9月24日
	 * @Description 读取三段式按钮的开关以及模式
	 */
	public int HRIF_ReadTriStageSwitch(int boxID,int rbtID,IntData nThreeStageEnable,IntData nThreeStageMode) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadTriStageSwitch_base(rbtID,nThreeStageEnable,nThreeStageMode);
	}

	/**
	 * @Title HRIF_SetTriStageSwitch
	 * @param rbtID   机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2024年9月24日
	 * @Description 设置三段式按钮的开关以及模式
	 */
	public int HRIF_SetTriStageSwitch(int boxID,int rbtID,int nThreeStageEnable,int nThreeStageMode) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetTriStageSwitch_base(rbtID, nThreeStageEnable,nThreeStageMode);
	}

	/**
	 * @Title HRIF_GetLoadIdentifyResult
	 * @param rbtID   机器人ID
	 * @param boxID   电箱ID
	 * @param status   负载辨识状态
	 * @param progress   进度状态
	 * @param mass   负载质量
	 * @param cx   质心X坐标
	 * @param cy   质心Y坐标
	 * @param cz   质心Z坐标
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2025年3月20日
	 * @Description 获取负载辨识结果
	 */
	public int HRIF_GetLoadIdentifyResult(int boxID,int rbtID,IntData status,IntData progress,DoubleData mass,DoubleData cx,DoubleData cy,DoubleData cz) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GetLoadIdentifyResult_base(rbtID, status,progress,mass,cx,cy,cz);
	}

	/**
	 * @Title HRIF_LoadIdentify
	 * @param rbtID   机器人ID
	 * @param boxID   电箱ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2025年3月20日
	 * @Description 开始负载辨识结果
	 */
	public int HRIF_LoadIdentify(int boxID,int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_LoadIdentify_base(rbtID);
	}

	/**
	 * @Title HRIF_ClearCoControlExpandAxisPos
	 * @param rbtID   机器人ID
	 * @param boxID   电箱ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2025年3月21日
	 * @Description 清除联动扩展轴位置(设联动扩展轴当前位置为零点)
	 */
	public int HRIF_ClearCoControlExpandAxisPos(int boxID,int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ClearCoControlExpandAxisPos_base(rbtID);
	}


	/**-----------------------Part3 脚本控制指令-------------------------*/

	/**
	 * @Title HRIF_RunFunc
	 * @date 2022年6月1日
 	 * @param boxID 电箱ID
	 * @param strFuncName 函数名称
	 * @param param 参数表
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 运行指定脚本函数
	 */
	public int HRIF_RunFunc(int boxID,String strFuncName, Vector<String> param)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_RunFunc_base(strFuncName, param);
	}

	/**
	 * @Title HRIF_StartScript
	 * @date 2022年6月1日
 	 * @param boxID 电箱ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 执行脚本 Main函数，调用后执行示教器页面编译好的脚本文件 main函数
	 */
	public int HRIF_StartScript(int boxID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_StartScript_base();
	}

	/**
	 * @Title HRIF_StopScript
	 * @date 2022年6月1日
 	 * @param boxID 电箱ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 停止脚本，调用后停止示教器页面正在执行脚本文件
	 */
	public int HRIF_StopScript(int boxID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_StopScript_base();
	}

	/**
	 * @Title HRIF_PauseScript
	 * @date 2022年6月1日
 	 * @param boxID 电箱ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 暂停脚本，调用后暂停示教器页面正在执行脚本文件
	 */
	public int HRIF_PauseScript(int boxID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PauseScript_base();
	}

	/**
	 * @Title HRIF_ContinueScript
	 * @date 2022年6月1日
 	 * @param boxID 电箱ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 继续脚本，调用后继续运行示教器页面正在暂停的脚本文件，不处于暂停状态则返回 20018错误
	 */
	public int HRIF_ContinueScript(int boxID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ContinueScript_base();
	}

	/**
	 * @Title HRIF_SwitchScript
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 应用指定的脚本程序。
	 */
	public int HRIF_SwitchScript(int boxID,int nRbtID,String sScriptName) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SwitchScript_base(nRbtID, sScriptName);
	}

	/**
	 * @Title HRIF_ReadDefaultScript
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 读取系统当前应用的脚本程序的名字。
	 */
	public int HRIF_ReadDefaultScript(int boxID,int nRbtID,StringData sScriptName) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadDefaultScript_base(nRbtID, sScriptName);
	}

	/**-----------------------Part4 电箱控制指令-------------------------*/

	/**
	 * @Title HRIF_ReadBoxInfo
	 * @date 2022年6月1日
  	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nConnected 电箱连接状态
	 * @param n48V_ON 48V电压状态
	 * @param db48OUT_Voltag 48V输出电压值
	 * @param db48OUT_Current 48V输出电流值
	 * @param nRemoteBTN 远程急停状态
	 * @param nThreeStageBTN 三段按钮状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取电箱信息
	 */
	public int HRIF_ReadBoxInfo(int boxID, int rbtID, IntData nConnected, IntData n48V_ON, DoubleData db48OUT_Voltag,
			DoubleData db48OUT_Current, IntData nRemoteBTN, IntData nThreeStageBTN,DoubleData db12V_Voltag,DoubleData db12V_Current)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadBoxInfo_base(rbtID, nConnected, n48V_ON, db48OUT_Voltag,
			 db48OUT_Current, nRemoteBTN, nThreeStageBTN,db12V_Voltag,db12V_Current);
	}

	/**
	 * @Title HRIF_ReadBoxCI
	 * @date 2022年6月1日
  	 * @param boxID 电箱ID
	 * @param nBit 控制数字输入位
	 * @param nVal 控制数字输入对应位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取电箱控制数字输入状态
	 */
	public int HRIF_ReadBoxCI(int boxID, int nBit, IntData nVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadBoxCI_base(nBit, nVal);
	}

	/**
	 * @Title HRIF_ReadBoxDI
	 * @date 2022年6月1日
  	 * @param boxID 电箱ID
	 * @param nBit 通用数字输入位
	 * @param nVal 控制数字输入对应位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取电箱通用数字输入状态
	 */
	public int HRIF_ReadBoxDI(int boxID, int nBit, IntData nVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadBoxDI_base(nBit, nVal);
	}

	/**
	 * @Title HRIF_ReadBoxCO
	 * @date 2022年6月1日
  	 * @param boxID 电箱ID
	 * @param nBit 控制数字输出位
	 * @param nVal 控制数字输入对应位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取电箱控制数字输出状态
	 */
	public int HRIF_ReadBoxCO(int boxID, int nBit, IntData nVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadBoxCO_base(nBit, nVal);
	}

	/**
	 * @Title HRIF_ReadBoxDO
	 * @date 2022年6月1日
  	 * @param boxID 电箱ID
	 * @param nBit 控制数字输出位
	 * @param nVal 控制数字输入对应位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取电箱通用数字输出状态
	 */
	public int HRIF_ReadBoxDO(int boxID, int nBit, IntData nVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadBoxDO_base(nBit, nVal);
	}

	/**
	 * @Title HRIF_ReadBoxAI
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param nBit 模拟量输入通道
	 * @param dVal 对应的模拟量通道值
	 * @return int,0:调用成功;>0:返回错误码
	 * @Description 读取电箱模拟量输入值
	 */
	public int HRIF_ReadBoxAI(int boxID, int nBit, DoubleData dVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadBoxAI_base(nBit, dVal);
	}

	/**
	 * @Title HRIF_ReadBoxAO
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param nBit 模拟量输出通道
	 * @param nMode 对应的模拟量通道模式
	 * @param dVal 对应的模拟量通道值
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取电箱模拟量输出值
	 */
	public int HRIF_ReadBoxAO(int boxID, int nBit, IntData nMode, DoubleData dVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadBoxAO_base(nBit, nMode, dVal);
	}

	/**
	 * @Title HRIF_SetBoxCO
	 * @date 2022年6月1日
 	 * @param boxID 电箱ID
	 * @param nBit 控制数字输出位
	 * @param nVal 控制数字输出状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置电箱控制数字输出状态
	 */
	public int HRIF_SetBoxCO(int boxID, int nBit, int nVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetBoxCO_base(nBit, nVal);
	}

	/**
	 * @Title HRIF_SetBoxDO
	 * @date 2022年6月1日
 	 * @param boxID 电箱ID
	 * @param nBit 通用数字输出位
	 * @param nVal 通用数字输出状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置电箱通用数字输出状态
	 */
	public int HRIF_SetBoxDO(int boxID, int nBit, int nVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetBoxDO_base(nBit, nVal);
	}

	/**
	 * @Title HRIF_SetBoxAOMode
	 * @date 2022年6月1日
  	 * @param boxID 电箱ID
	 * @param nBit 模拟量通道
	 * @param nVal 模式
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置电箱模拟量输出模式
	 */
	public int HRIF_SetBoxAOMode(int boxID, int nBit, int nVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetBoxAOMode_base(nBit, nVal);
	}

	/**
	 * @Title HRIF_SetBoxAOVal
	 * @date 2022年6月1日
  	 * @param boxID 电箱ID
	 * @param nBit 模拟量通道
	 * @param dVal 对应模拟量值
	 * @param nMode 模式
	 * @return int,0:调用成功;>0:返回错误码
	 * @Description 设置电箱模拟量输出值
	 */
	public int HRIF_SetBoxAOVal(int boxID, int nBit, double dVal, int nMode) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetBoxAOVal_base(nBit, dVal, nMode);
	}

	/**
	 * @Title HRIF_SetEndDO
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nBit 末端数字输出位
	 * @param nVal 末端数字输出状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置末端数字输出状态
	 */
	public int HRIF_SetEndDO(int boxID, int rbtID, int nBit, int nVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetEndDO_base(rbtID, nBit, nVal);
	}

	/**
	 * @Title HRIF_ReadEndDI
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nBit 末端数字输入位
	 * @param nVal 末端数字输入对应位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取末端数字输入状态
	 */
	public int HRIF_ReadEndDI(int boxID, int rbtID, int nBit, IntData nVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadEndDI_base(rbtID, nBit, nVal);
	}

	/**
	 * @Title HRIF_SetMoveParamsAO
	 * @date 2024年9月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取末端数字输入状态
	 */
	public int HRIF_SetMoveParamsAO(int boxID, int rbtID, int nState, int nIndex,double dInitAO,double dWeldingAO) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetMoveParamsAO_base(rbtID,nState,nIndex,dInitAO,dWeldingAO);
	}

	/**
	 * @Title HRIF_ReadEndDO
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nBit 末端数字输出位
	 * @param nVal 末端数字输出对应位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取末端数字输出状态
	 */
	public int HRIF_ReadEndDO(int boxID, int rbtID, int nBit, IntData nVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadEndDO_base(rbtID, nBit, nVal);
	}

	/**
	 * @Title HRIF_ReadEndAI
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nBit 模拟量输入通道
	 * @param dVal 对应的模拟量通道值
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取末端模拟量输入值
	 */
	public int HRIF_ReadEndAI(int boxID, int rbtID, int nBit, DoubleData dVal) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadEndAI_base(rbtID, nBit, dVal);
	}

	/**
	 * @Title HRIF_cdsSetIO
	 * @date 2024年9月5日
	 * @param boxID 电箱ID
	 * @param nEndDOMask 需要更改的EndDO按bit标识
	 * @param nEndDOVal 各个需要更改的EndDO的目标状态
	 * @param nBoxDOMask 需要更改的BoxDO按bit标识
	 * @param nBoxDOVal 各个需要更改的BoxDO的目标状态
	 * @param nBoxCOMask 需要更改的BoxCO按bit标识
	 * @param nBoxCOVal 各个需要更改的BoxCO的目标状态
	 * @param nBoxAOCH0_Mask BoxAOCH0是否需要更改的标识
	 * @param nBoxAOCH0_Mode 模式
	 * @param nBoxAOCH1_Mask BoxAOCH1是否需要更改的标识
	 * @param nBoxAOCH1_Mode 模式
	 * @param dbBoxAOCH0_Val 对应模拟量值
	 * @param dbBoxAOCH1_Val 对应模拟量值
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 在运动指令到达目标点位前设置IO。
	 */
	public int HRIF_cdsSetIO(int boxID,int nEndDOMask,int nEndDOVal,int nBoxDOMask,int nBoxDOVal,
							 int nBoxCOMask, int nBoxCOVal, int nBoxAOCH0_Mask, int nBoxAOCH0_Mode, int nBoxAOCH1_Mask,
							 int nBoxAOCH1_Mode, double dbBoxAOCH0_Val, double dbBoxAOCH1_Val) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_cdsSetIO_base(nEndDOMask, nEndDOVal, nBoxDOMask, nBoxDOVal, nBoxCOMask, nBoxCOVal, nBoxAOCH0_Mask, nBoxAOCH0_Mode,
		nBoxAOCH1_Mask, nBoxAOCH1_Mode, dbBoxAOCH0_Val, dbBoxAOCH1_Val);
	}

	/**
	 * @Title HRIF_ReadCIConfig
	 * @date 2024年9月6日
	 * @param boxID 电箱ID
	 * @param nBit 控制数字输出位
	 * @param nFunction 指定nBit的功能类型
	 * @param nTriggerType 功能的触发方式
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取CI的功能
	 */
	public int HRIF_ReadCIConfig(int boxID, int nBit,IntData nFunction,IntData nTriggerType) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCIConfig_base(nBit,nFunction,nTriggerType);
	}
	public int HRIF_ReadCIConfig(int boxID,IntData nFunction0,IntData nFunction1,IntData nFunction2,
								 IntData nFunction3,IntData nFunction4,IntData nFunction5,IntData nFunction6,
								 IntData nFunction7) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadAllCIConfig_base(nFunction0, nFunction1,nFunction2,nFunction3,nFunction4,nFunction5,nFunction6,nFunction7);
	}

	/**
	 * @Title HRIF_SetCIConfig
	 * @date 2024年9月6日
	 * @param boxID 电箱ID
	 * @param nFunction 功能类型
	 * @param nBit 控制数字输出位
	 * @param nTriggerType 触发方式
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置CI的功能
	 */
	public int HRIF_SetCIConfig(int boxID, int nBit, int nFunction, int nTriggerType) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetCIConfig_base(nBit, nFunction, nTriggerType);
	}
	public int HRIF_SetCIConfig(int boxID, int nBit, int nFunction) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		int defaultTriggerType = -1; // 设置默认值，替换成你需要的默认值
		return m_client[boxID].HRIF_SetCIConfig_base( nBit, nFunction, defaultTriggerType);
	}

	/**
	 * @Title HRIF_SetCOConfig
	 * @date 2024年9月6日
	 * @param boxID 电箱ID
	 * @param nBit 控制数字输出位
	 * @param nFunction 功能类型
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置CO的功能
	 */
	public int HRIF_SetCOConfig(int boxID, int nBit, int nFunction)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetCOConfig_base(nBit, nFunction);
	}

	/**
	 * @Title HRIF_ReadCOConfig
	 * @date 2024年9月6日
	 * @param boxID 电箱ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取CO的功能
	 */
	public int HRIF_ReadCOConfig(int boxID,IntData nFunction0,IntData nFunction1,IntData nFunction2,
								 IntData nFunction3,IntData nFunction4,IntData nFunction5,IntData nFunction6,
								 IntData nFunction7) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCOConfig_base(nFunction0, nFunction1,nFunction2,nFunction3,nFunction4,nFunction5,nFunction6,nFunction7);
	}

	/**
	 * @Title HRIF_EnableEndBTN
	 * @date 2024年9月11日
	 * @param boxID 电箱ID
	 * @param rbtID 控制数字输出位
	 * @param nStatus 模块按钮状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 启用或关闭末端
	 */
	public int HRIF_EnableEndBTN(int boxID,int rbtID, int nStatus)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_EnableEndBTN_base(rbtID, nStatus);
	}

	/**
	 * @Title HRIF_ReadEndBTN
	 * @date 2022年6月20日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nBit1 按键1状态
	 * @param nBit2 按键2状态
	 * @param nBit3 按键3状态
	 * @param nBit4  按键4状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取末端按键状态,根据搭载的末端类型,各状态表示含义会有区别
	 */
	public int HRIF_ReadEndBTN(int boxID, int rbtID, IntData nBit1,IntData nBit2,IntData nBit3,IntData nBit4)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadEndBTN_base(rbtID, nBit1, nBit2, nBit3, nBit4);
	}


	/**-----------------------Part5 状态读取与设置指令-------------------------*/

	/**
	 * @Title HRIF_SetOverride
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dOverride 速度比
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置速度比
	 */
	public int HRIF_SetOverride(int boxID, int rbtID, double dOverride) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetOverride_base(rbtID, dOverride);
	}

	/**
	 * @Title HRIF_SetToolMotion
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nState 状态
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 开启或关闭 Tool坐标系运动模式
	 */
	public int HRIF_SetToolMotion(int boxID, int rbtID, int nState) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetToolMotion_base(rbtID, nState);
	}

	/**
	 * @Title HRIF_SetPlayload
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dMass 质量
	 * @param dX 质心X方向偏移
	 * @param dY 质心Y方向偏移
	 * @param dZ 质心Z方向偏移
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置当前负载参数
	 */
	public int HRIF_SetPayload(int boxID, int rbtID, double dMass, double dX, double dY, double dZ)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetPayload_base(rbtID, dMass, dX, dY, dZ);
	}

	/**
	 * @Title HRIF_SetJointMaxVel
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dJ1 J1轴最大速度，单位[°/s]
	 * @param dJ2 J2轴最大速度，单位[°/s]
	 * @param dJ3 J3轴最大速度，单位[°/s]
	 * @param dJ4 J4轴最大速度，单位[°/s]
	 * @param dJ5 J5轴最大速度，单位[°/s]
	 * @param dJ6 J6轴最大速度，单位[°/s]
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置关节最大运动速度
	 */
	public int HRIF_SetJointMaxVel(int boxID, int rbtID, double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetJointMaxVel_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_SetJointMaxVel_nj
	 * @date 2024年10月11日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置关节最大运动速度
	 */
	public int HRIF_SetJointMaxVel_nj(int boxID, int rbtID,JointsDatas joints)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetJointMaxVel_nj_base(rbtID, joints);
	}

	/**
	 * @Title HRIF_SetJointMaxAcc
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dJ1 J1轴最大加速度，单位[°/ s^2]
	 * @param dJ2 J2轴最大加速度，单位[°/ s^2]
	 * @param dJ3 J3轴最大加速度，单位[°/ s^2]
	 * @param dJ4 J4轴最大加速度，单位[°/ s^2]
	 * @param dJ5 J5轴最大加速度，单位[°/ s^2]
	 * @param dJ6 J6轴最大加速度，单位[°/ s^2]
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置关节最大运动加速度
	 */
	public int HRIF_SetJointMaxAcc(int boxID, int rbtID, double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetJointMaxAcc_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_SetJointMaxAcc_nj
	 * @date 2024年10月11日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置关节最大运动加速度
	 */
	public int HRIF_SetJointMaxAcc_nj(int boxID, int rbtID,JointsDatas joints)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetJointMaxAcc_nj_base(rbtID, joints);
	}


	/**
	 * @Title HRIF_SetLinearMaxVel
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dMaxVel 最大直线速度，默认[500], 单位[mm/s]
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置直线运动最大速度
	 */
	public int HRIF_SetLinearMaxVel(int boxID, int rbtID, double dMaxVel) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetLinearMaxVel_base(rbtID, dMaxVel);
	}

	/**
	 * @Title HRIF_SetLinearMaxAcc
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dMaxAcc 最大直线加速度，默认[2500], 单位[mm/s^2]
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置直线运动最大加速度
	 */
	public int HRIF_SetLinearMaxAcc(int boxID, int rbtID, double dMaxAcc) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetLinearMaxAcc_base(rbtID, dMaxAcc);
	}

	/**
	 * @Title HRIF_SetMaxAcsRange
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dMaxJ1 关节1最大范围
	 * @param dMaxJ2 关节2最大范围
	 * @param dMaxJ3 关节3最大范围
	 * @param dMaxJ4 关节4最大范围
	 * @param dMaxJ5 关节5最大范围
	 * @param dMaxJ6 关节6最大范围
	 * @param dMinJ1 关节1最小范围
	 * @param dMinJ2 关节2最小范围
	 * @param dMinJ3 关节3最小范围
	 * @param dMinJ4 关节4最小范围
	 * @param dMinJ5 关节5最小范围
	 * @param dMinJ6 关节6最小范围
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置关节最大运动范围
	 * @Attention 设置关节最大运动范围
	 */
	public int HRIF_SetMaxAcsRange(int boxID, int rbtID, double dMaxJ1, double dMaxJ2,double dMaxJ3,double dMaxJ4,
			double dMaxJ5,double dMaxJ6,double dMinJ1,double dMinJ2,double dMinJ3,double dMinJ4,double dMinJ5,double dMinJ6)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetMaxAcsRange_base(rbtID, dMaxJ1, dMaxJ2, dMaxJ3, dMaxJ4, dMaxJ5, dMaxJ6,
															dMinJ1, dMinJ2, dMinJ3, dMinJ4, dMinJ5, dMinJ6);
	}

	/**
	 * @Title HRIF_SetMaxAcsRange_nj
	 * @date 2024年10月11日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置关节最大运动范围
	 */
	public int HRIF_SetMaxAcsRange_nj(int boxID, int rbtID,JointsDatas jointsmax,JointsDatas jointsmin)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetMaxAcsRange_nj_base(rbtID, jointsmax,jointsmin);
	}


	/**
	 * @Title HRIF_SetMaxPcsRange
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dMaxX X最大边界
	 * @param dMaxY Y最大边界
	 * @param dMaxZ Z最大边界
	 * @param dMaxRx Rx最大边界
	 * @param dMaxRy Ry最大边界
	 * @param dMaxRz Rz最大边界
	 * @param dMinX X最小边界
	 * @param dMinY Y最小边界
	 * @param dMinZ Z最小边界
	 * @param dMinRx Rx最小边界
	 * @param dMinRy Ry最小边界
	 * @param dMinRz Rz最小边界
	 * @param dUcs_X 用户坐标X
	 * @param dUcs_Y 用户坐标Y
	 * @param dUcs_Z 用户坐标Z
	 * @param dUcs_Rx 用户坐标Rx
	 * @param dUcs_Ry 用户坐标Ry
	 * @param dUcs_Rz 用户坐标Rz
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置空间最大运动范围
	 * @Attention 控制器重启后失效，需要重新设置
	 */
	public int HRIF_SetMaxPcsRange(int boxID, int rbtID, double dMaxX, double dMaxY, double dMaxZ,double dMaxRx,double dMaxRy,double dMaxRz,
			double dMinX, double dMinY,double dMinZ,double dMinRx,double dMinRy,double dMinRz,
			double dUcs_X,double dUcs_Y,double dUcs_Z,double dUcs_Rx,double dUcs_Ry,double dUcs_Rz) {

		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetMaxPcsRange_base(rbtID,  dMaxX,  dMaxY,  dMaxZ, dMaxRx, dMaxRy, dMaxRz,
				dMinX,  dMinY, dMinZ, dMinRx, dMinRy, dMinRz,
				dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);


	}

	/**
	 * @Title HRIF_SetCollideLevel
	 * @date 2024年9月5日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nSafeLevel 设置安全风险等级
	 * @return int, 0 调用成功;>0 返回错误码
	 */
	public int HRIF_SetCollideLevel(int boxID, int rbtID, int nSafeLevel) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetCollideLevel_base( rbtID, nSafeLevel);
	}

	/**
	 * @Title HRIF_ReadMaxPayload
	 * @date 2024年9月5日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dbMaxPayload 读取末端最大负载
	 * @return int, 0 调用成功;>0 返回错误码
	 */
	public int HRIF_ReadMaxPayload(int boxID, int rbtID, DoubleData dbMaxPayload) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadMaxPayload_base( rbtID, dbMaxPayload);
	}

	/**
	 * @Title HRIF_ReadPayload
	 * @date 2024年9月5日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dMass 质量
	 * @param dX 质心X方向偏移
	 * @param dY 质心Y方向偏移
	 * @param dZ 质心Z方向偏移
	 * @return int, 0 调用成功;>0 返回错误码
	 */
	public int HRIF_ReadPayload(int boxID, int rbtID, DoubleData dMass, DoubleData dX, DoubleData dY, DoubleData dZ) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadPayload_base( rbtID, dMass,dX,dY,dZ);
	}


	/**
	 * @Title HRIF_ReadOverride
	 * @date 2023年2月11日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dOverride 速度比返回值
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取速度比
	 */
	public int HRIF_ReadOverride(int boxID, int rbtID, DoubleData dOverride) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadOverride_base( rbtID, dOverride);
	}

	/**
	 * @Title HRIF_ReadJointMaxVel
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dJ1 J1轴最大速度
	 * @param dJ2 J2轴最大速度
	 * @param dJ3 J3轴最大速度
	 * @param dJ4 J4轴最大速度
	 * @param dJ5 J5轴最大速度
	 * @param dJ6 J6轴最大速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节最大运动速度
	 */
	public int HRIF_ReadJointMaxVel(int boxID, int rbtID, DoubleData dJ1, DoubleData dJ2, DoubleData dJ3, DoubleData dJ4, DoubleData dJ5, DoubleData dJ6)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadJointMaxVel_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_ReadJointMaxVel_nj
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节最大运动速度
	 */
	public int HRIF_ReadJointMaxVel_nj(int boxID, int rbtID,JointsData joints)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadJointMaxVel_nj_base(rbtID, joints);
	}

	/**
	 * @Title HRIF_ReadJointMaxAcc
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
 	 * @param dJ1 J1轴最大加速度
	 * @param dJ2 J2轴最大加速度
	 * @param dJ3 J3轴最大加速度
	 * @param dJ4 J4轴最大加速度
	 * @param dJ5 J5轴最大加速度
	 * @param dJ6 J6轴最大加速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节最大运动加速度
	 */
	public int HRIF_ReadJointMaxAcc(int boxID, int rbtID, DoubleData dJ1, DoubleData dJ2, DoubleData dJ3, DoubleData dJ4, DoubleData dJ5, DoubleData dJ6)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadJointMaxAcc_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_ReadJointMaxAcc_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节最大运动加速度
	 */
	public int HRIF_ReadJointMaxAcc_nj(int boxID, int rbtID, JointsData joints)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadJointMaxAcc_nj_base(rbtID, joints);
	}

	/**
	 * @Title HRIF_ReadJointMaxJerk
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
 	 * @param dJ1 J1轴最大加加速度
	 * @param dJ2 J2轴最大加加速度
	 * @param dJ3 J3轴最大加加速度
	 * @param dJ4 J4轴最大加加速度
	 * @param dJ5 J5轴最大加加速度
	 * @param dJ6 J6轴最大加加速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节最大运动加加速度
	 */
	public int HRIF_ReadJointMaxJerk(int boxID, int rbtID, DoubleData dJ1, DoubleData dJ2, DoubleData dJ3, DoubleData dJ4, DoubleData dJ5, DoubleData dJ6)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadJointMaxJerk_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_ReadJointMaxJerk_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节最大运动加加速度
	 */
	public int HRIF_ReadJointMaxJerk_nj(int boxID, int rbtID,JointsData joints)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadJointMaxJerk_nj_base(rbtID, joints);
	}

	/**
	 * @Title HRIF_ReadLinearMaxSpeed
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dMaxVel 最大直线速度
	 * @param dMaxAcc 最大直线加速度
	 * @param dMaxJerk 最大直线加加速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取直线运动最大速度参数
	 */
	public int HRIF_ReadLinearMaxSpeed(int boxID, int rbtID, DoubleData dMaxVel, DoubleData dMaxAcc, DoubleData dMaxJerk) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadLinearMaxSpeed_base(rbtID, dMaxVel, dMaxAcc, dMaxJerk);
	}

	/**
	 * @Title HRIF_ReadEmergencyInfo
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nESTO_Error 急停错误
	 * @param nESTO 急停信号
	 * @param nSafeGuard_Error 安全光幕错误
	 * @param nSafeGuard 安全光幕信号
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取急停信息
	 */
	public int HRIF_ReadEmergencyInfo(int boxID, int rbtID, IntData nESTO_Error, IntData nESTO, IntData nSafeGuard_Error, IntData nSafeGuard)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadEmergencyInfo_base(rbtID, nESTO_Error, nESTO, nSafeGuard_Error, nSafeGuard);
	}

	/**
	 * @Title HRIF_ReadRobotState
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nMovingState 运动中状态
	 * @param nEnableState 已使能状态
	 * @param nErrorState 错误状态
	 * @param nErrorCode 错误码
	 * @param nErrorAxis 错误轴ID
	 * @param nBreaking 开抱闸状态
	 * @param nPause 暂停状态
	 * @param nEmergencyStop 急停状态
	 * @param nSafeGuard 安全光幕状态
	 * @param nElectrify 已上电状态
	 * @param nIsConnectToBox 电箱连接状态
	 * @param nBlendingDone 路点运动状态
	 * @param nInPos 到位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前机器人状态标志
	 */
	public int HRIF_ReadRobotState(int boxID, int rbtID, IntData nMovingState,IntData nEnableState,IntData nErrorState,IntData nErrorCode,
			IntData nErrorAxis,IntData nBreaking,IntData nPause,IntData nEmergencyStop,IntData nSafeGuard,
			IntData nElectrify,IntData nIsConnectToBox, IntData nBlendingDone, IntData nInPos) {

		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadRobotState_base(rbtID, nMovingState, nEnableState, nErrorState, nErrorCode,
				 nErrorAxis, nBreaking, nPause, nEmergencyStop, nSafeGuard,
				 nElectrify, nIsConnectToBox,  nBlendingDone,  nInPos);
	}

	/**
	 * @Title HRIF_ReadRobotFlags
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nMovingState 运动中状态
	 * @param nEnableState 已使能状态
	 * @param nErrorState 错误状态
	 * @param nErrorCode 错误码
	 * @param nErrorAxis 错误轴ID
	 * @param nBreaking 开抱闸状态
	 * @param nPause 暂停状态
	 * @param nBlendingDone 路点运动状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前机器人状态标志
	 */
	public int HRIF_ReadRobotFlags(int boxID, int rbtID, IntData nMovingState,IntData nEnableState,IntData nErrorState,IntData nErrorCode,
			IntData nErrorAxis,IntData nBreaking,IntData nPause, IntData nBlendingDone) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadRobotFlags_base(rbtID, nMovingState, nEnableState, nErrorState, nErrorCode,
				 nErrorAxis, nBreaking, nPause,  nBlendingDone);
	}

	/**
	 * @Title HRIF_ReadCurWaypointID
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param strCurWaypointID 当前ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取WayPoint当前运动 ID号
	 */
	public int HRIF_ReadCurWaypointID(int boxID, int rbtID, StringData strCurWaypointID)
	{
		return m_client[boxID].HRIF_ReadCurWaypointID_base(rbtID, strCurWaypointID);
	}

	/**
	 * @Title HRIF_ReadAxisErrorCode
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nErrorCode 当前错误码
	 * @param nJ1 J1轴错误码
	 * @param nJ2 J2轴错误码
	 * @param nJ3 J3轴错误码
	 * @param nJ4 J4轴错误码
	 * @param nJ5 J5轴错误码
	 * @param nJ6 J6轴错误码
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取错误码
	 */
	public int HRIF_ReadAxisErrorCode(int boxID, int rbtID, IntData nErrorCode, IntData nJ1,IntData nJ2,IntData nJ3,IntData nJ4,IntData nJ5,IntData nJ6)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadAxisErrorCode_base(rbtID, nErrorCode, nJ1, nJ2, nJ3, nJ4, nJ5, nJ6);
	}

	/**
	 * @Title HRIF_ReadAxisErrorCode_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nErrorCode 当前错误码
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取错误码
	 */
	public int HRIF_ReadAxisErrorCode_nj(int boxID, int rbtID, IntData nErrorCode,  JointsData jointsData)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadAxisErrorCode_nj_base(rbtID, nErrorCode, jointsData);
	}

	/**
	 * @Title HRIF_ReadCurFSM
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nCurFSM 当前状态机
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前状态机
	 */
	public int HRIF_ReadCurFSM(int boxID, int rbtID, IntData nCurFSM ) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCurFSM_base(rbtID, nCurFSM);
	}

	/**
	 * @Title HRIF_ReadCurFSMFromCPS
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nCurFSM 当前状态机
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前状态机
	 */
	public int HRIF_ReadCurFSMFromCPS(int boxID, int rbtID, IntData nCurFSM) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCurFSMFromCPS_base(rbtID, nCurFSM);
	}

	/**
	 * @Title HRIF_ReadPointByName
	 * @date 2023年2月16日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param pointName 点位名称
	 * @param dJ1 关节1坐标
	 * @param dJ2 关节2坐标
	 * @param dJ3 关节3坐标
	 * @param dJ4 关节4坐标
	 * @param dJ5 关节5坐标
	 * @param dJ6 关节6坐标
	 * @param dX X坐标
	 * @param dY Y坐标
	 * @param dZ Z坐标
	 * @param dRx Rx坐标
	 * @param dRy Ry坐标
	 * @param dRz Rz坐标
	 * @param dTcp_X Tcp_X坐标
	 * @param dTcp_Y Tcp_Y坐标
	 * @param dTcp_Z Tcp_Z坐标
	 * @param dTcp_Rx Tcp_Rx坐标
	 * @param dTcp_Ry Tcp_Ry坐标
	 * @param dTcp_Rz Tcp_Rz坐标
	 * @param dUcs_X Ucs_X坐标
	 * @param dUcs_Y Ucs_Y坐标
	 * @param dUcs_Z Ucs_Z坐标
	 * @param dUcs_Rx Ucs_Rx坐标
	 * @param dUcs_Ry Ucs_Ry坐标
	 * @param dUcs_Rz Ucs_Rz坐标
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 根据点位名称读取点位信息
	 */
	public int HRIF_ReadPointByName(int boxID, int rbtID, String pointName,DoubleData dJ1, DoubleData dJ2, DoubleData dJ3,
			DoubleData dJ4, DoubleData dJ5, DoubleData dJ6, DoubleData dX, DoubleData dY, DoubleData dZ, DoubleData dRx, DoubleData dRy, DoubleData dRz,
			DoubleData dTcp_X, DoubleData dTcp_Y, DoubleData dTcp_Z, DoubleData dTcp_Rx, DoubleData dTcp_Ry, DoubleData dTcp_Rz,
			DoubleData dUcs_X, DoubleData dUcs_Y, DoubleData dUcs_Z, DoubleData dUcs_Rx, DoubleData dUcs_Ry, DoubleData dUcs_Rz) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadPointByName_base(rbtID,pointName,dJ1,dJ2,dJ3,dJ4,dJ5,dJ6,dX,dY,dZ,dRx,dRy,dRz,
				dTcp_X,dTcp_Y,dTcp_Z,dTcp_Rx, dTcp_Ry, dTcp_Rz,dUcs_X,dUcs_Y,dUcs_Z,dUcs_Rx,dUcs_Ry,dUcs_Rz);
	}

	/**
	 * @Title HRIF_ReadPointByName_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param pointName 点位名称
	 * @param dX X坐标
	 * @param dY Y坐标
	 * @param dZ Z坐标
	 * @param dRx Rx坐标
	 * @param dRy Ry坐标
	 * @param dRz Rz坐标
	 * @param dTcp_X Tcp_X坐标
	 * @param dTcp_Y Tcp_Y坐标
	 * @param dTcp_Z Tcp_Z坐标
	 * @param dTcp_Rx Tcp_Rx坐标
	 * @param dTcp_Ry Tcp_Ry坐标
	 * @param dTcp_Rz Tcp_Rz坐标
	 * @param dUcs_X Ucs_X坐标
	 * @param dUcs_Y Ucs_Y坐标
	 * @param dUcs_Z Ucs_Z坐标
	 * @param dUcs_Rx Ucs_Rx坐标
	 * @param dUcs_Ry Ucs_Ry坐标
	 * @param dUcs_Rz Ucs_Rz坐标
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 根据点位名称读取点位信息
	 */
	public int HRIF_ReadPointByName_nj(int boxID, int rbtID, String pointName, JointsData joints, DoubleData dX, DoubleData dY, DoubleData dZ, DoubleData dRx, DoubleData dRy, DoubleData dRz,
									DoubleData dTcp_X, DoubleData dTcp_Y, DoubleData dTcp_Z, DoubleData dTcp_Rx, DoubleData dTcp_Ry, DoubleData dTcp_Rz,
									DoubleData dUcs_X, DoubleData dUcs_Y, DoubleData dUcs_Z, DoubleData dUcs_Rx, DoubleData dUcs_Ry, DoubleData dUcs_Rz) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadPointByName_nj_base(rbtID,pointName, joints,dX,dY,dZ,dRx,dRy,dRz,
				dTcp_X,dTcp_Y,dTcp_Z,dTcp_Rx, dTcp_Ry, dTcp_Rz,dUcs_X,dUcs_Y,dUcs_Z,dUcs_Rx,dUcs_Ry,dUcs_Rz);
	}

	/**
	 * @Title HRIF_ReadPointList
	 * @date 2024年9月23日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取系统中保存的点位名称列表
	 */
	public int HRIF_ReadPointList(int boxID, int rbtID, StringData  PointList) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadPointList_base(rbtID, PointList);
	}

	/**-----------------------Part6 位置/速度/电流读取指令-------------------------*/

	/**
	 * @Title HRIF_ReadActPos
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
 	 * @param dX 迪卡尔坐标X
	 * @param dY 迪卡尔坐标Y
	 * @param dZ 迪卡尔坐标Z
	 * @param dRx 迪卡尔坐标Rx
	 * @param dRy 迪卡尔坐标Ry
	 * @param dRz 迪卡尔坐标Rz
 	 * @param dJ1 J1关节坐标
	 * @param dJ2 J2关节坐标
	 * @param dJ3 J3关节坐标
	 * @param dJ4 J4关节坐标
	 * @param dJ5 J5关节坐标
	 * @param dJ6 J6关节坐标
	 * @param dTCP_X 当前工具坐标X
	 * @param dTCP_Y 当前工具坐标Y
	 * @param dTCP_Z 当前工具坐标Z
	 * @param dTCP_Rx 当前工具坐标Rx
	 * @param dTCP_Ry 当前工具坐标Ry
	 * @param dTCP_Rz 当前工具坐标Rz
	 * @param dUCS_X 当前用户坐标X
	 * @param dUCS_Y 当前用户坐标Y
	 * @param dUCS_Z 当前用户坐标Z
	 * @param dUCS_Rx 当前用户坐标Rx
	 * @param dUCS_Ry 当前用户坐标Ry
	 * @param dUCS_Rz 当前用户坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前实际位置信息
	 */
	public int HRIF_ReadActPos(int boxID, int rbtID, DoubleData dX, DoubleData dY, DoubleData dZ, DoubleData dRx, DoubleData dRy, DoubleData dRz,
			DoubleData dJ1, DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6,
			DoubleData dTCP_X, DoubleData dTCP_Y,DoubleData dTCP_Z,DoubleData dTCP_Rx,DoubleData dTCP_Ry,DoubleData dTCP_Rz,
			DoubleData dUCS_X, DoubleData dUCS_Y,DoubleData dUCS_Z,DoubleData dUCS_Rx,DoubleData dUCS_Ry,DoubleData dUCS_Rz)
	{

		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadActPos_base(rbtID, dX, dY, dZ, dRx, dRy, dRz,
			dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
			dTCP_X, dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz,
			dUCS_X, dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz);
	}

	/**
	 * @Title HRIF_ReadActPos_nj
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX 迪卡尔坐标X
	 * @param dY 迪卡尔坐标Y
	 * @param dZ 迪卡尔坐标Z
	 * @param dRx 迪卡尔坐标Rx
	 * @param dRy 迪卡尔坐标Ry
	 * @param dRz 迪卡尔坐标Rz
	 * @param dTCP_X 当前工具坐标X
	 * @param dTCP_Y 当前工具坐标Y
	 * @param dTCP_Z 当前工具坐标Z
	 * @param dTCP_Rx 当前工具坐标Rx
	 * @param dTCP_Ry 当前工具坐标Ry
	 * @param dTCP_Rz 当前工具坐标Rz
	 * @param dUCS_X 当前用户坐标X
	 * @param dUCS_Y 当前用户坐标Y
	 * @param dUCS_Z 当前用户坐标Z
	 * @param dUCS_Rx 当前用户坐标Rx
	 * @param dUCS_Ry 当前用户坐标Ry
	 * @param dUCS_Rz 当前用户坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前实际位置信息
	 */
	public int HRIF_ReadActPos_nj(int boxID, int rbtID, DoubleData dX, DoubleData dY, DoubleData dZ, DoubleData dRx, DoubleData dRy, DoubleData dRz,
								  JointsData joints,
							   DoubleData dTCP_X, DoubleData dTCP_Y,DoubleData dTCP_Z,DoubleData dTCP_Rx,DoubleData dTCP_Ry,DoubleData dTCP_Rz,
							   DoubleData dUCS_X, DoubleData dUCS_Y,DoubleData dUCS_Z,DoubleData dUCS_Rx,DoubleData dUCS_Ry,DoubleData dUCS_Rz)
	{

		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadActPos_nj_base(rbtID, dX, dY, dZ, dRx, dRy, dRz,
				 joints,
				dTCP_X, dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz,
				dUCS_X, dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz);
	}

	/**
	 * @Title HRIF_ReadCmdJointPos
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dJ1 J1关节坐标
	 * @param dJ2 J2关节坐标
	 * @param dJ3 J3关节坐标
	 * @param dJ4 J4关节坐标
	 * @param dJ5 J5关节坐标
	 * @param dJ6 J6关节坐标
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节命令位置
	 */
	public int HRIF_ReadCmdJointPos(int boxID,int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCmdJointPos_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_ReadCmdJointPos_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节命令位置
	 */
	public int HRIF_ReadCmdJointPos_nj(int boxID,int rbtID,JointsData joints) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCmdJointPos_nj_base(rbtID, joints);
	}

	/**
	 * @Title HRIF_ReadActJointPos
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dJ1 J1关节坐标
	 * @param dJ2 J2关节坐标
	 * @param dJ3 J3关节坐标
	 * @param dJ4 J4关节坐标
	 * @param dJ5 J5关节坐标
	 * @param dJ6 J6关节坐标
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节实际位置
	 */
	public int HRIF_ReadActJointPos(int boxID,int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadActJointPos_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_ReadActJointPos_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节实际位置
	 */
	public int HRIF_ReadActJointPos_nj(int boxID,int rbtID,JointsData joints) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadActJointPos_nj_base(rbtID, joints);
	}

	/**
	 * @Title HRIF_ReadCmdTcpPos
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX TCP坐标X
	 * @param dY TCP坐标Y
	 * @param dZ TCP坐标Z
	 * @param dRx TCP坐标Rx
	 * @param dRy TCP坐标Ry
	 * @param dRz TCP坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取命令TCP位置
	 */
	public int HRIF_ReadCmdTcpPos(int boxID, int rbtID, DoubleData dX,DoubleData dY,DoubleData dZ,DoubleData dRx,DoubleData dRy,DoubleData dRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCmdTcpPos_base(rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_ReadActTcpPos
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
 	 * @param dX TCP坐标X
	 * @param dY TCP坐标Y
	 * @param dZ TCP坐标Z
	 * @param dRx TCP坐标Rx
	 * @param dRy TCP坐标Ry
	 * @param dRz TCP坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取实际 TCP位置
	 */
	public int HRIF_ReadActTcpPos(int boxID, int rbtID, DoubleData dX,DoubleData dY,DoubleData dZ,DoubleData dRx,DoubleData dRy,DoubleData dRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadActTcpPos_base(rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_ReadCmdJointVel
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dJ1 J1轴速度
	 * @param dJ2 J2轴速度
	 * @param dJ3 J3轴速度
	 * @param dJ4 J4轴速度
	 * @param dJ5 J5轴速度
	 * @param dJ6 J6轴速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节命令速度
	 */
	public int HRIF_ReadCmdJointVel(int boxID,int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCmdJointVel_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_ReadCmdJointVel_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节命令速度
	 */
	public int HRIF_ReadCmdJointVel_nj(int boxID,int rbtID,JointsData joints) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCmdJointVel_nj_base(rbtID, joints);
	}

	/**
	 * @Title HRIF_ReadActJointVel
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dJ1 J1轴速度
	 * @param dJ2 J2轴速度
	 * @param dJ3 J3轴速度
	 * @param dJ4 J4轴速度
	 * @param dJ5 J5轴速度
	 * @param dJ6 J6轴速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节实际速度
	 */
	public int HRIF_ReadActJointVel(int boxID,int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadActJointVel_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_ReadActJointVel_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节实际速度
	 */
	public int HRIF_ReadActJointVel_nj(int boxID,int rbtID,JointsData joints) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadActJointVel_nj_base(rbtID, joints);
	}

	/**
	 * @Title HRIF_ReadCmdTcpVel
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX TCP速度X
	 * @param dY TCP速度Y
	 * @param dZ TCP速度Z
	 * @param dRx TCP速度Rx
	 * @param dRy TCP速度Ry
	 * @param dRz TCP速度Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取命令TCP速度
	 */
	public int HRIF_ReadCmdTcpVel(int boxID, int rbtID, DoubleData dX,DoubleData dY,DoubleData dZ,DoubleData dRx,DoubleData dRy,DoubleData dRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCmdTcpVel_base(rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_ReadActTcpVel
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX TCP速度X
	 * @param dY TCP速度Y
	 * @param dZ TCP速度Z
	 * @param dRx TCP速度Rx
	 * @param dRy TCP速度Ry
	 * @param dRz TCP速度Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取实际TCP速度
	 */
	public int HRIF_ReadActTcpVel(int boxID, int rbtID, DoubleData dX,DoubleData dY,DoubleData dZ,DoubleData dRx,DoubleData dRy,DoubleData dRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadActTcpVel_base(rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_ReadCmdJointCur
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dJ1 J1关节电流
	 * @param dJ2 J2关节电流
	 * @param dJ3 J3关节电流
	 * @param dJ4 J4关节电流
	 * @param dJ5 J5关节电流
	 * @param dJ6 J6关节电流
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节命令电流
	 */
	public int HRIF_ReadCmdJointCur(int boxID,int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCmdJointCur_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_ReadCmdJointCur_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节命令电流
	 */
	public int HRIF_ReadCmdJointCur_nj(int boxID,int rbtID,JointsData joints) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCmdJointCur_nj_base(rbtID, joints);
	}

	/**
	 * @Title HRIF_ReadActJointCur
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
 	 * @param dJ1 J1关节电流
	 * @param dJ2 J2关节电流
	 * @param dJ3 J3关节电流
	 * @param dJ4 J4关节电流
	 * @param dJ5 J5关节电流
	 * @param dJ6 J6关节电流
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节实际电流
	 */
	public int HRIF_ReadActJointCur(int boxID,int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadActJointCur_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_ReadActJointCur_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节实际电流
	 */
	public int HRIF_ReadActJointCur_nj(int boxID,int rbtID,JointsData joints) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadActJointCur_nj_base(rbtID, joints);
	}

	/**
	 * @Title HRIF_ReadTcpVelocity
	 * @date 2022年6月1日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dCmdVel 命令速度
	 * @param dActVel 实际速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取TCP末端速度
	 */
	public int HRIF_ReadTcpVelocity(int boxID, int rbtID, DoubleData dCmdVel, DoubleData dActVel) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadTcpVelocity_base(rbtID, dCmdVel,dActVel);
	}

	/**
	 * @Title HRIF_SetRotationVelocityControlMode
	 * @date 2025年3月21日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nMode 0 : 受限(默认)模式  1 : 不受限模式
	 * @return int, 0 调用成功;>0 返回错误码
	 */
	public int HRIF_SetRotationVelocityControlMode(int boxID, int rbtID, int nMode) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetRotationVelocityControlMode_base( rbtID, nMode);
	}

	/**-----------------------Part7 坐标转换计算指令-------------------------*/

	/**
	 * @Title HRIF_Quaternion2RPY
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dQuaW QW
	 * @param dQuaX QX
	 * @param dQuaY QY
	 * @param dQuaZ QZ
	 * @param dRx Rx
	 * @param dRy Ry
	 * @param dRz Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 四元素转欧拉角
	 */
	public int HRIF_Quaternion2RPY(int boxID, int rbtID, double dQuaW, double dQuaX, double dQuaY, double dQuaZ,
			DoubleData dRx, DoubleData dRy, DoubleData dRz) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_Quaternion2RPY_base(rbtID, dQuaW, dQuaX, dQuaY, dQuaZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_RPY2Quaternion
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param Rx Rx
	 * @param Ry Ry
	 * @param Rz Rz
 	 * @param dQuaW QW
	 * @param dQuaX QX
	 * @param dQuaY QY
	 * @param dQuaZ QZ
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 欧拉角转四元素
	 */
	public int HRIF_RPY2Quaternion(int boxID, int rbtID, double Rx, double Ry, double Rz,
			DoubleData dQuaW, DoubleData dQuaX,DoubleData dQuaY,DoubleData dQuaZ)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_RPY2Quaternion_base(rbtID, Rx, Ry, Rz, dQuaW, dQuaX, dQuaY, dQuaZ);
	}

	/**
	 * @Title HRIF_GetInverseKin
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
 	 * @param dCoord_X 需要计算逆解的目标迪卡尔位置X
	 * @param dCoord_Y 需要计算逆解的目标迪卡尔位置Y
	 * @param dCoord_Z 需要计算逆解的目标迪卡尔位置Z
	 * @param dCoord_Rx 需要计算逆解的目标迪卡尔位置Rx
	 * @param dCoord_Ry 需要计算逆解的目标迪卡尔位置Ry
	 * @param dCoord_Rz 需要计算逆解的目标迪卡尔位置Rz
	 * @param dTCP_X 工具坐标X
	 * @param dTCP_Y 工具坐标Y
	 * @param dTCP_Z 工具坐标Z
	 * @param dTCP_Rx 工具坐标Rx
	 * @param dTCP_Ry 工具坐标Ry
	 * @param dTCP_Rz 工具坐标Rz
	 * @param dUCS_X 用户坐标X
	 * @param dUCS_Y 用户坐标Y
	 * @param dUCS_Z 用户坐标Z
	 * @param dUCS_Rx 用户坐标Rx
	 * @param dUCS_Ry 用户坐标Ry
	 * @param dUCS_Rz 用户坐标Rz
 	 * @param dJ1 参考关节坐标J1
	 * @param dJ2 参考关节坐标J2
	 * @param dJ3 参考关节坐标J3
	 * @param dJ4 参考关节坐标J4
	 * @param dJ5 参考关节坐标J5
	 * @param dJ6 参考关节坐标J6
	 * @param dTargetJ1 输出的关节坐标J1
	 * @param dTargetJ2 输出的关节坐标J2
	 * @param dTargetJ3 输出的关节坐标J3
	 * @param dTargetJ4 输出的关节坐标J4
	 * @param dTargetJ5 输出的关节坐标J5
	 * @param dTargetJ6 输出的关节坐标J6
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 逆解,由指定用户坐标系位置和工具坐标系下的迪卡尔坐标计算对应的关节坐标位置
	 */
	public int HRIF_GetInverseKin(int boxID, int rbtID, double dCoord_X, double dCoord_Y, double dCoord_Z, double dCoord_Rx,double dCoord_Ry, double dCoord_Rz,
			double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
			double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz,
			double dJ1,double dJ2,double dJ3,double dJ4,double dJ5,double dJ6,
			DoubleData dTargetJ1,DoubleData dTargetJ2,DoubleData dTargetJ3,DoubleData dTargetJ4,DoubleData dTargetJ5,DoubleData dTargetJ6)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GetInverseKin_base(rbtID, dCoord_X, dCoord_Y, dCoord_Z, dCoord_Rx, dCoord_Ry, dCoord_Rz,
				dTCP_X, dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz,
				dUCS_X, dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz,
				dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
				dTargetJ1, dTargetJ2, dTargetJ3, dTargetJ4, dTargetJ5, dTargetJ6);
	}

	/**
	 * @Title HRIF_GetInverseKin_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dCoord_X 需要计算逆解的目标迪卡尔位置X
	 * @param dCoord_Y 需要计算逆解的目标迪卡尔位置Y
	 * @param dCoord_Z 需要计算逆解的目标迪卡尔位置Z
	 * @param dCoord_Rx 需要计算逆解的目标迪卡尔位置Rx
	 * @param dCoord_Ry 需要计算逆解的目标迪卡尔位置Ry
	 * @param dCoord_Rz 需要计算逆解的目标迪卡尔位置Rz
	 * @param dTCP_X 工具坐标X
	 * @param dTCP_Y 工具坐标Y
	 * @param dTCP_Z 工具坐标Z
	 * @param dTCP_Rx 工具坐标Rx
	 * @param dTCP_Ry 工具坐标Ry
	 * @param dTCP_Rz 工具坐标Rz
	 * @param dUCS_X 用户坐标X
	 * @param dUCS_Y 用户坐标Y
	 * @param dUCS_Z 用户坐标Z
	 * @param dUCS_Rx 用户坐标Rx
	 * @param dUCS_Ry 用户坐标Ry
	 * @param dUCS_Rz 用户坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 逆解,由指定用户坐标系位置和工具坐标系下的迪卡尔坐标计算对应的关节坐标位置
	 */
	public int HRIF_GetInverseKin_nj(int boxID, int rbtID, double dCoord_X, double dCoord_Y, double dCoord_Z, double dCoord_Rx,double dCoord_Ry, double dCoord_Rz,
								  double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
								  double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz,
								  JointsDatas joints , JointsData Target)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GetInverseKin_nj_base(rbtID, dCoord_X, dCoord_Y, dCoord_Z, dCoord_Rx, dCoord_Ry, dCoord_Rz,
				dTCP_X, dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz,
				dUCS_X, dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz, joints, Target);
	}

	/**
	 * @Title HRIF_GetForwardKin
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dJ1 关节坐标J1
	 * @param dJ2 关节坐标J2
	 * @param dJ3 关节坐标J3
	 * @param dJ4 关节坐标J4
	 * @param dJ5 关节坐标J5
	 * @param dJ6 关节坐标J6
	 * @param dTCP_X 工具坐标X
	 * @param dTCP_Y 工具坐标Y
	 * @param dTCP_Z 工具坐标Z
	 * @param dTCP_Rx 工具坐标Rx
	 * @param dTCP_Ry 工具坐标Ry
	 * @param dTCP_Rz 工具坐标Rz
	 * @param dUCS_X 用户坐标X
	 * @param dUCS_Y 用户坐标Y
	 * @param dUCS_Z 用户坐标Z
	 * @param dUCS_Rx 用户坐标Rx
	 * @param dUCS_Ry 用户坐标Ry
	 * @param dUCS_Rz 用户坐标Rz
	 * @param dTargetX 目标迪卡尔坐标X
	 * @param dTargetY 目标迪卡尔坐标Y
	 * @param dTargetZ 目标迪卡尔坐标Z
	 * @param dTargetRx 目标迪卡尔坐标Rx
	 * @param dTargetRy 目标迪卡尔坐标Ry
	 * @param dTargetRz 目标迪卡尔坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 正解,由关节坐标位置计算指定用户坐标系和工具坐标系下的迪卡尔坐标位置
	 */
	public int HRIF_GetForwardKin(int boxID, int rbtID, double dJ1,double dJ2,double dJ3,double dJ4,double dJ5,double dJ6,
			double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
			double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz,
			DoubleData dTargetX,DoubleData dTargetY,DoubleData dTargetZ,DoubleData dTargetRx,DoubleData dTargetRy,DoubleData dTargetRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GetForwardKin_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
			 dTCP_X, dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz,
			 dUCS_X, dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz,
			 dTargetX, dTargetY, dTargetZ, dTargetRx, dTargetRy, dTargetRz);
	}

	/**
	 * @Title HRIF_GetForwardKin_nj
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dTCP_X 工具坐标X
	 * @param dTCP_Y 工具坐标Y
	 * @param dTCP_Z 工具坐标Z
	 * @param dTCP_Rx 工具坐标Rx
	 * @param dTCP_Ry 工具坐标Ry
	 * @param dTCP_Rz 工具坐标Rz
	 * @param dUCS_X 用户坐标X
	 * @param dUCS_Y 用户坐标Y
	 * @param dUCS_Z 用户坐标Z
	 * @param dUCS_Rx 用户坐标Rx
	 * @param dUCS_Ry 用户坐标Ry
	 * @param dUCS_Rz 用户坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 正解,由关节坐标位置计算指定用户坐标系和工具坐标系下的迪卡尔坐标位置
	 */
	public int HRIF_GetForwardKin_nj(int boxID, int rbtID,
								  double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
								  double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz, JointsDatas joints,
								  DoubleData dTargetX,DoubleData dTargetY,DoubleData dTargetZ,DoubleData dTargetRx,DoubleData dTargetRy,DoubleData dTargetRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GetForwardKin_nj_base(rbtID,
				dTCP_X, dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz,
				dUCS_X, dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz, joints,
				dTargetX, dTargetY, dTargetZ, dTargetRx, dTargetRy, dTargetRz);
	}

	/**
	 * @Title HRIF_Base2UcsTcp
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
  	 * @param dCoord_X 迪卡尔位置X
	 * @param dCoord_Y 迪卡尔位置Y
	 * @param dCoord_Z 迪卡尔位置Z
	 * @param dCoord_Rx 迪卡尔位置Rx
	 * @param dCoord_Ry 迪卡尔位置Ry
	 * @param dCoord_Rz 迪卡尔位置Rz
 	 * @param dTCP_X 工具坐标X
	 * @param dTCP_Y 工具坐标Y
	 * @param dTCP_Z 工具坐标Z
	 * @param dTCP_Rx 工具坐标Rx
	 * @param dTCP_Ry 工具坐标Ry
	 * @param dTCP_Rz 工具坐标Rz
	 * @param dUCS_X 用户坐标X
	 * @param dUCS_Y 用户坐标Y
	 * @param dUCS_Z 用户坐标Z
	 * @param dUCS_Rx 用户坐标Rx
	 * @param dUCS_Ry 用户坐标Ry
	 * @param dUCS_Rz 用户坐标Rz
	 * @param dTargetX 目标迪卡尔坐标X
	 * @param dTargetY 目标迪卡尔坐标Y
	 * @param dTargetZ 目标迪卡尔坐标Z
	 * @param dTargetRx 目标迪卡尔坐标Rx
	 * @param dTargetRy 目标迪卡尔坐标Ry
	 * @param dTargetRz 目标迪卡尔坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 由基坐标系下的坐标位置计算指定用户坐标系和工具坐标系下的迪卡尔坐标位置
	 */
	public int HRIF_Base2UcsTcp(int boxID, int rbtID, double dCoord_X, double dCoord_Y,double dCoord_Z,double dCoord_Rx,double dCoord_Ry,double dCoord_Rz,
			double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
			double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz,
			DoubleData dTargetX,DoubleData dTargetY,DoubleData dTargetZ,DoubleData dTargetRx,DoubleData dTargetRy,DoubleData dTargetRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_Base2UcsTcp_base(rbtID, dCoord_X, dCoord_Y, dCoord_Z, dCoord_Rx, dCoord_Ry, dCoord_Rz,
			 dTCP_X,  dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz,
			 dUCS_X,  dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz,
			 dTargetX, dTargetY, dTargetZ, dTargetRx, dTargetRy, dTargetRz);
	}

	/**
	 * @Title HRIF_UcsTcp2Base
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
  	 * @param dCoord_X 迪卡尔位置X
	 * @param dCoord_Y 迪卡尔位置Y
	 * @param dCoord_Z 迪卡尔位置Z
	 * @param dCoord_Rx 迪卡尔位置Rx
	 * @param dCoord_Ry 迪卡尔位置Ry
	 * @param dCoord_Rz 迪卡尔位置Rz
 	 * @param dTCP_X 工具坐标X
	 * @param dTCP_Y 工具坐标Y
	 * @param dTCP_Z 工具坐标Z
	 * @param dTCP_Rx 工具坐标Rx
	 * @param dTCP_Ry 工具坐标Ry
	 * @param dTCP_Rz 工具坐标Rz
	 * @param dUCS_X 用户坐标X
	 * @param dUCS_Y 用户坐标Y
	 * @param dUCS_Z 用户坐标Z
	 * @param dUCS_Rx 用户坐标Rx
	 * @param dUCS_Ry 用户坐标Ry
	 * @param dUCS_Rz 用户坐标Rz
	 * @param dTargetX 目标迪卡尔坐标X
	 * @param dTargetY 目标迪卡尔坐标Y
	 * @param dTargetZ 目标迪卡尔坐标Z
	 * @param dTargetRx 目标迪卡尔坐标Rx
	 * @param dTargetRy 目标迪卡尔坐标Ry
	 * @param dTargetRz 目标迪卡尔坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 由指定用户坐标系和工具坐标系下的迪卡尔坐标位置计算基坐标系下的坐标位置
	 */
	public int HRIF_UcsTcp2Base(int boxID, int rbtID, double dCoord_X, double dCoord_Y,double dCoord_Z,double dCoord_Rx,double dCoord_Ry,double dCoord_Rz,
			double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
			double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz,
			DoubleData dTargetX,DoubleData dTargetY,DoubleData dTargetZ,DoubleData dTargetRx,DoubleData dTargetRy,DoubleData dTargetRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_UcsTcp2Base_base(rbtID, dCoord_X, dCoord_Y, dCoord_Z, dCoord_Rx, dCoord_Ry, dCoord_Rz,
			 dTCP_X,  dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz,
			 dUCS_X,  dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz,
			 dTargetX, dTargetY, dTargetZ, dTargetRx, dTargetRy, dTargetRz);
	}

	/**
	 * @Title HRIF_PoseAdd
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dPose1_X 空间坐标1-X
	 * @param dPose1_Y 空间坐标1-Y
	 * @param dPose1_Z 空间坐标1-Z
	 * @param dPose1_Rx 空间坐标1-Rx
	 * @param dPose1_Ry 空间坐标1-Ry
	 * @param dPose1_Rz 空间坐标1-Rz
	 * @param dPose2_X  空间坐标2-X
	 * @param dPose2_Y 空间坐标2-Y
	 * @param dPose2_Z 空间坐标2-Z
	 * @param dPose2_Rx 空间坐标2-Rx
	 * @param dPose2_Ry 空间坐标2-Ry
	 * @param dPose2_Rz 空间坐标2-Rz
	 * @param dPose3_X 目标迪卡尔坐标X
	 * @param dPose3_Y 目标迪卡尔坐标Y
	 * @param dPose3_Z 目标迪卡尔坐标Z
	 * @param dPose3_Rx 目标迪卡尔坐标Rx
	 * @param dPose3_Ry 目标迪卡尔坐标Ry
	 * @param dPose3_Rz 目标迪卡尔坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 点位加法计算
	 */
	public int HRIF_PoseAdd(int boxID, int rbtID, double dPose1_X, double dPose1_Y,double dPose1_Z,double dPose1_Rx,double dPose1_Ry,double dPose1_Rz,
			double dPose2_X, double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,
			DoubleData dPose3_X, DoubleData dPose3_Y,DoubleData dPose3_Z,DoubleData dPose3_Rx,DoubleData dPose3_Ry,DoubleData dPose3_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PoseAdd_base(rbtID,  dPose1_X,  dPose1_Y, dPose1_Z, dPose1_Rx, dPose1_Ry, dPose1_Rz,
			 dPose2_X,  dPose2_Y, dPose2_Z, dPose2_Rx, dPose2_Ry, dPose2_Rz,
			 dPose3_X,  dPose3_Y, dPose3_Z, dPose3_Rx, dPose3_Ry, dPose3_Rz);
	}

	/**
	 * @Title HRIF_PoseSub
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dPose1_X 空间坐标1-X
	 * @param dPose1_Y 空间坐标1-Y
	 * @param dPose1_Z 空间坐标1-Z
	 * @param dPose1_Rx 空间坐标1-Rx
	 * @param dPose1_Ry 空间坐标1-Ry
	 * @param dPose1_Rz 空间坐标1-Rz
	 * @param dPose2_X  空间坐标2-X
	 * @param dPose2_Y 空间坐标2-Y
	 * @param dPose2_Z 空间坐标2-Z
	 * @param dPose2_Rx 空间坐标2-Rx
	 * @param dPose2_Ry 空间坐标2-Ry
	 * @param dPose2_Rz 空间坐标2-Rz
	 * @param dPose3_X 目标迪卡尔坐标X
	 * @param dPose3_Y 目标迪卡尔坐标Y
	 * @param dPose3_Z 目标迪卡尔坐标Z
	 * @param dPose3_Rx 目标迪卡尔坐标Rx
	 * @param dPose3_Ry 目标迪卡尔坐标Ry
	 * @param dPose3_Rz 目标迪卡尔坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 点位减法计算
	 */
	public int HRIF_PoseSub(int boxID, int rbtID, double dPose1_X, double dPose1_Y,double dPose1_Z,double dPose1_Rx,double dPose1_Ry,double dPose1_Rz,
			double dPose2_X, double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,
			DoubleData dPose3_X, DoubleData dPose3_Y,DoubleData dPose3_Z,DoubleData dPose3_Rx,DoubleData dPose3_Ry,DoubleData dPose3_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PoseSub_base(rbtID,  dPose1_X,  dPose1_Y, dPose1_Z, dPose1_Rx, dPose1_Ry, dPose1_Rz,
			 dPose2_X,  dPose2_Y, dPose2_Z, dPose2_Rx, dPose2_Ry, dPose2_Rz,
			 dPose3_X,  dPose3_Y, dPose3_Z, dPose3_Rx, dPose3_Ry, dPose3_Rz);
	}

	/**
	 * @Title HRIF_PoseTrans
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dPose1_X 空间坐标1-X
	 * @param dPose1_Y 空间坐标1-Y
	 * @param dPose1_Z 空间坐标1-Z
	 * @param dPose1_Rx 空间坐标1-Rx
	 * @param dPose1_Ry 空间坐标1-Ry
	 * @param dPose1_Rz 空间坐标1-Rz
	 * @param dPose2_X  空间坐标2-X
	 * @param dPose2_Y 空间坐标2-Y
	 * @param dPose2_Z 空间坐标2-Z
	 * @param dPose2_Rx 空间坐标2-Rx
	 * @param dPose2_Ry 空间坐标2-Ry
	 * @param dPose2_Rz 空间坐标2-Rz
	 * @param dPose3_X 目标迪卡尔坐标X
	 * @param dPose3_Y 目标迪卡尔坐标Y
	 * @param dPose3_Z 目标迪卡尔坐标Z
	 * @param dPose3_Rx 目标迪卡尔坐标Rx
	 * @param dPose3_Ry 目标迪卡尔坐标Ry
	 * @param dPose3_Rz 目标迪卡尔坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 坐标变换
	 */
	public int HRIF_PoseTrans(int boxID, int rbtID, double dPose1_X, double dPose1_Y,double dPose1_Z,double dPose1_Rx,double dPose1_Ry,double dPose1_Rz,
			double dPose2_X, double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,
			DoubleData dPose3_X, DoubleData dPose3_Y,DoubleData dPose3_Z,DoubleData dPose3_Rx,DoubleData dPose3_Ry,DoubleData dPose3_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PoseTrans_base(rbtID,  dPose1_X,  dPose1_Y, dPose1_Z, dPose1_Rx, dPose1_Ry, dPose1_Rz,
			 dPose2_X,  dPose2_Y, dPose2_Z, dPose2_Rx, dPose2_Ry, dPose2_Rz,
			 dPose3_X,  dPose3_Y, dPose3_Z, dPose3_Rx, dPose3_Ry, dPose3_Rz);
	}

	/**
	 * @Title HRIF_PoseInverse
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dPose1_X 空间坐标1-X
	 * @param dPose1_Y 空间坐标1-Y
	 * @param dPose1_Z 空间坐标1-Z
	 * @param dPose1_Rx 空间坐标1-Rx
	 * @param dPose1_Ry 空间坐标1-Ry
	 * @param dPose1_Rz 空间坐标1-Rz
	 * @param dPose3_X 目标迪卡尔坐标X
	 * @param dPose3_Y 目标迪卡尔坐标Y
	 * @param dPose3_Z 目标迪卡尔坐标Z
	 * @param dPose3_Rx 目标迪卡尔坐标Rx
	 * @param dPose3_Ry 目标迪卡尔坐标Ry
	 * @param dPose3_Rz 目标迪卡尔坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 坐标逆变换
	 */
	public int HRIF_PoseInverse(int boxID, int rbtID, double dPose1_X, double dPose1_Y,double dPose1_Z,double dPose1_Rx,double dPose1_Ry,double dPose1_Rz,
			DoubleData dPose3_X, DoubleData dPose3_Y,DoubleData dPose3_Z,DoubleData dPose3_Rx,DoubleData dPose3_Ry,DoubleData dPose3_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PoseInverse_base(rbtID, dPose1_X, dPose1_Y, dPose1_Z, dPose1_Rx, dPose1_Ry, dPose1_Rz,
				 dPose3_X, dPose3_Y, dPose3_Z, dPose3_Rx, dPose3_Ry, dPose3_Rz);
	}

	/**
	 * @Title HRIF_PoseDist
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dPose1_X 空间坐标1-X
	 * @param dPose1_Y 空间坐标1-Y
	 * @param dPose1_Z 空间坐标1-Z
	 * @param dPose1_Rx 空间坐标1-Rx
	 * @param dPose1_Ry 空间坐标1-Ry
	 * @param dPose1_Rz 空间坐标1-Rz
	 * @param dPose2_X  空间坐标2-X
	 * @param dPose2_Y 空间坐标2-Y
	 * @param dPose2_Z 空间坐标2-Z
	 * @param dPose2_Rx 空间坐标2-Rx
	 * @param dPose2_Ry 空间坐标2-Ry
	 * @param dPose2_Rz 空间坐标2-Rz
	 * @param dDistance 点位距离
	 * @param dAngle 姿态距离
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 计算点位距离
	 */
	public int HRIF_PoseDist(int boxID, int rbtID, double dPose1_X, double dPose1_Y,double dPose1_Z,double dPose1_Rx,double dPose1_Ry,double dPose1_Rz,
			double dPose2_X, double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,
			DoubleData dDistance, DoubleData  dAngle)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PoseDist_base(rbtID, dPose1_X, dPose1_Y, dPose1_Z, dPose1_Rx, dPose1_Ry, dPose1_Rz,
			 dPose2_X,  dPose2_Y, dPose2_Z, dPose2_Rx, dPose2_Ry, dPose2_Rz,
			 dDistance, dAngle);
	}

	/**
	 * @Title HRIF_PoseInterpolate
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dPose1_X 空间坐标1-X
	 * @param dPose1_Y 空间坐标1-Y
	 * @param dPose1_Z 空间坐标1-Z
	 * @param dPose1_Rx 空间坐标1-Rx
	 * @param dPose1_Ry 空间坐标1-Ry
	 * @param dPose1_Rz 空间坐标1-Rz
	 * @param dPose2_X  空间坐标2-X
	 * @param dPose2_Y 空间坐标2-Y
	 * @param dPose2_Z 空间坐标2-Z
	 * @param dPose2_Rx 空间坐标2-Rx
	 * @param dPose2_Ry 空间坐标2-Ry
	 * @param dPose2_Rz 空间坐标2-Rz
	 * @param dAlpha 插补比例
 	 * @param dPose3_X 目标迪卡尔坐标X
	 * @param dPose3_Y 目标迪卡尔坐标Y
	 * @param dPose3_Z 目标迪卡尔坐标Z
	 * @param dPose3_Rx 目标迪卡尔坐标Rx
	 * @param dPose3_Ry 目标迪卡尔坐标Ry
	 * @param dPose3_Rz 目标迪卡尔坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 空间位置直线插补计算
	 */
	public int HRIF_PoseInterpolate(int boxID, int rbtID, double dPose1_X, double dPose1_Y,double dPose1_Z,double dPose1_Rx,double dPose1_Ry,double dPose1_Rz,
			double dPose2_X, double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz, double dAlpha,
			DoubleData dPose3_X, DoubleData dPose3_Y,DoubleData dPose3_Z,DoubleData dPose3_Rx,DoubleData dPose3_Ry,DoubleData dPose3_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PoseInterpolate_base(rbtID, dPose1_X, dPose1_Y, dPose1_Z, dPose1_Rx, dPose1_Ry, dPose1_Rz,
				 dPose2_X,  dPose2_Y, dPose2_Z, dPose2_Rx, dPose2_Ry, dPose2_Rz, dAlpha,
				 dPose3_X,  dPose3_Y, dPose3_Z, dPose3_Rx, dPose3_Ry, dPose3_Rz);
	}

	/**
	 * @Title HRIF_PoseDefdFrame
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dPose1_X 空间坐标1-X
	 * @param dPose1_Y 空间坐标1-Y
	 * @param dPose1_Z 空间坐标1-Z
	 * @param dPose2_X 空间坐标2-X
	 * @param dPose2_Y 空间坐标2-Y
	 * @param dPose2_Z 空间坐标2-Z
	 * @param dPose3_X 空间坐标3-X
	 * @param dPose3_Y 空间坐标3-Y
	 * @param dPose3_Z 空间坐标3-Z
	 * @param dPose4_X 空间坐标4-X
	 * @param dPose4_Y 空间坐标4-Y
	 * @param dPose4_Z 空间坐标4-Z
	 * @param dPose5_X 空间坐标5-X
	 * @param dPose5_Y 空间坐标5-Y
	 * @param dPose5_Z 空间坐标5-Z
	 * @param dPose6_X 空间坐标6-X
	 * @param dPose6_Y 空间坐标6-Y
	 * @param dPose6_Z 空间坐标6-Z
	 * @param dUcs_X 计算结果X
	 * @param dUcs_Y 计算结果Y
	 * @param dUcs_Z 计算结果Z
	 * @param dUcs_Rx 计算结果Rx
	 * @param dUcs_Ry 计算结果Ry
	 * @param dUcs_Rz 计算结果Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 描述：以轨迹中心旋转计算， p1,p2,p3为旋转前轨迹的特征点， p4,p5,p6为旋转后的轨迹的特征点
	 */
	public int HRIF_PoseDefdFrame(int boxID, int rbtID, double dPose1_X,double dPose1_Y,double dPose1_Z,
			double dPose2_X,double dPose2_Y,double dPose2_Z,
			double dPose3_X,double dPose3_Y,double dPose3_Z,
			double dPose4_X,double dPose4_Y,double dPose4_Z,
			double dPose5_X,double dPose5_Y,double dPose5_Z,
			double dPose6_X,double dPose6_Y,double dPose6_Z,
			DoubleData dUcs_X,DoubleData dUcs_Y,DoubleData dUcs_Z,DoubleData dUcs_Rx,DoubleData dUcs_Ry,DoubleData dUcs_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PoseDefdFrame_base(rbtID, dPose1_X, dPose1_Y, dPose1_Z,
				 dPose2_X, dPose2_Y, dPose2_Z,
				 dPose3_X, dPose3_Y, dPose3_Z,
				 dPose4_X, dPose4_Y, dPose4_Z,
				 dPose5_X, dPose5_Y, dPose5_Z,
				 dPose6_X, dPose6_Y, dPose6_Z,
				 dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
	}

	/**
	 * @Title HRIF_CalUcsLine
	 * @param boxID       电箱ID
	 * @param rbtID       机器人ID
	 * @param dPose1_X    空间坐标1-X
	 * @param dPose1_Y    空间坐标1-Y
	 * @param dPose1_Z    空间坐标1-Z
	 * @param dPose2_X    空间坐标2-X
	 * @param dPose2_Y    空间坐标2-Y
	 * @param dPose2_Z    空间坐标2-Z
	 * @param dRetPose_X
	 * @param dRetPose_Y
	 * @param dRetPose_Z
	 * @param dRetPose_Rx
	 * @param dRetPose_Ry
	 * @param dRetPose_Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @date 2024年9月10日
	 * @Description 描述：通过两点直线法计算UCS
	 */
	public int HRIF_CalUcsLine(int boxID, int rbtID, double dPose1_X, double dPose1_Y , double dPose1_Z ,double dPose1_Rx, double dPose1_Ry , double dPose1_Rz ,
							   double dPose2_X , double dPose2_Y, double dPose2_Z ,double dPose2_Rx,double dPose2_Ry, double dPose2_Rz,
								  DoubleData dRetPose_X,DoubleData dRetPose_Y,DoubleData dRetPose_Z,DoubleData dRetPose_Rx,DoubleData dRetPose_Ry,DoubleData dRetPose_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_CalUcsLine_base(rbtID,  dPose1_X, dPose1_Y, dPose1_Z, dPose1_Rx, dPose1_Ry, dPose1_Rz, dPose2_X, dPose2_Y, dPose2_Z, dPose2_Rx, dPose2_Ry, dPose2_Rz,
				dRetPose_X, dRetPose_Y, dRetPose_Z, dRetPose_Rx, dRetPose_Ry, dRetPose_Rz);
	}

	/**
	 * @Title HRIF_CalUcsPlane
	 * @param boxID       电箱ID
	 * @param rbtID       机器人ID
	 * @param dPose1_X    空间坐标1-X
	 * @param dPose1_Y    空间坐标1-Y
	 * @param dPose1_Z    空间坐标1-Z
	 * @param dPose2_X    空间坐标2-X
	 * @param dPose2_Y    空间坐标2-Y
	 * @param dPose2_Z    空间坐标2-Z
	 * @param dPose3_X    空间坐标3-X
	 * @param dPose3_Y    空间坐标3-Y
	 * @param dPose3_Z    空间坐标3-Z
	 * @param dRetPose_X
	 * @param dRetPose_Y
	 * @param dRetPose_Z
	 * @param dRetPose_Rx
	 * @param dRetPose_Ry
	 * @param dRetPose_Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @date 2024年9月10日
	 * @Description 描述：通过三点平面法计算UCS
	 */
	public int HRIF_CalUcsPlane(int boxID, int rbtID, double dPose1_X,double dPose1_Y,double dPose1_Z,
								double dPose2_X,double dPose2_Y,double dPose2_Z,
								double dPose3_X,double dPose3_Y,double dPose3_Z,
								DoubleData dRetPose_X,DoubleData dRetPose_Y,DoubleData dRetPose_Z,DoubleData dRetPose_Rx,DoubleData dRetPose_Ry,DoubleData dRetPose_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_CalUcsPlane_base(rbtID, dPose1_X, dPose1_Y, dPose1_Z,
				dPose2_X, dPose2_Y, dPose2_Z,
				dPose3_X, dPose3_Y, dPose3_Z,
				dRetPose_X, dRetPose_Y, dRetPose_Z, dRetPose_Rx, dRetPose_Ry, dRetPose_Rz);
	}

	/**
	 * @Title HRIF_CalTcp3P
	 * @param boxID       电箱ID
	 * @param rbtID       机器人ID
	 * @param dPose1_X    空间坐标1-X
	 * @param dPose1_Y    空间坐标1-Y
	 * @param dPose1_Z    空间坐标1-Z
	 * @param dPose2_X    空间坐标2-X
	 * @param dPose2_Y    空间坐标2-Y
	 * @param dPose2_Z    空间坐标2-Z
	 * @param dPose3_X    空间坐标3-X
	 * @param dPose3_Y    空间坐标3-Y
	 * @param dPose3_Z    空间坐标3-Z
	 * @param dRetPose_X
	 * @param dRetPose_Y
	 * @param dRetPose_Z
	 * @param dRetPose_Rx
	 * @param dRetPose_Ry
	 * @param dRetPose_Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @date 2024年9月10日
	 * @Description 描述：通过三点平面法计算TCP
	 * */
	public int HRIF_CalTcp3P(int boxID, int rbtID, double dPose1_X, double dPose1_Y , double dPose1_Z ,double dPose1_Rx, double dPose1_Ry , double dPose1_Rz ,
							   double dPose2_X , double dPose2_Y, double dPose2_Z ,double dPose2_Rx,double dPose2_Ry, double dPose2_Rz,
							 double dPose3_X , double dPose3_Y , double dPose3_Z,double dPose3_Rx, double dPose3_Ry, double dPose3_Rz,
							   DoubleData dRetPose_X,DoubleData dRetPose_Y,DoubleData dRetPose_Z,DoubleData dRetPose_Rx,DoubleData dRetPose_Ry,DoubleData dRetPose_Rz,IntData quality )
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_CalTcp3P_base(rbtID,  dPose1_X, dPose1_Y, dPose1_Z, dPose1_Rx, dPose1_Ry, dPose1_Rz, dPose2_X, dPose2_Y, dPose2_Z, dPose2_Rx, dPose2_Ry, dPose2_Rz,
				dPose3_X, dPose3_Y, dPose3_Z, dPose3_Rx, dPose3_Ry, dPose3_Rz,
				dRetPose_X, dRetPose_Y, dRetPose_Z, dRetPose_Rx, dRetPose_Ry, dRetPose_Rz,quality );
	}

	/**
	 * @Title HRIF_CalTcp4P
	 * @param boxID       电箱ID
	 * @param rbtID       机器人ID
	 * @param dPose1_X    空间坐标1-X
	 * @param dPose1_Y    空间坐标1-Y
	 * @param dPose1_Z    空间坐标1-Z
	 * @param dPose2_X    空间坐标2-X
	 * @param dPose2_Y    空间坐标2-Y
	 * @param dPose2_Z    空间坐标2-Z
	 * @param dPose3_X    空间坐标3-X
	 * @param dPose3_Y    空间坐标3-Y
	 * @param dPose3_Z    空间坐标3-Z
	 * @param dRetPose_X
	 * @param dRetPose_Y
	 * @param dRetPose_Z
	 * @param dRetPose_Rx
	 * @param dRetPose_Ry
	 * @param dRetPose_Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @date 2024年9月10日
	 * @Description 描述：通过四点平面法计算TCP
	 */
	public int HRIF_CalTcp4P(int boxID, int rbtID, double dPose1_X, double dPose1_Y , double dPose1_Z ,double dPose1_Rx, double dPose1_Ry , double dPose1_Rz ,
							 double dPose2_X , double dPose2_Y, double dPose2_Z ,double dPose2_Rx,double dPose2_Ry, double dPose2_Rz,
							 double dPose3_X , double dPose3_Y , double dPose3_Z,double dPose3_Rx, double dPose3_Ry, double dPose3_Rz,
							 double dPose4_X, double dPose4_Y, double dPose4_Z,double dPose4_Rx , double dPose4_Ry, double dPose4_Rz,
							 DoubleData dRetPose_X,DoubleData dRetPose_Y,DoubleData dRetPose_Z,DoubleData dRetPose_Rx,DoubleData dRetPose_Ry,DoubleData dRetPose_Rz,
							 IntData quality,IntData errorIndex_P1,IntData errorIndex_P2,IntData errorIndex_P3,IntData errorIndex_P4)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_CalTcp4P_base(rbtID,  dPose1_X, dPose1_Y, dPose1_Z, dPose1_Rx, dPose1_Ry, dPose1_Rz, dPose2_X, dPose2_Y, dPose2_Z, dPose2_Rx, dPose2_Ry, dPose2_Rz,
				dPose3_X, dPose3_Y, dPose3_Z, dPose3_Rx, dPose3_Ry, dPose3_Rz,
				dPose4_X, dPose4_Y, dPose4_Z, dPose4_Rx, dPose4_Ry, dPose4_Rz,
				dRetPose_X, dRetPose_Y, dRetPose_Z, dRetPose_Rx, dRetPose_Ry, dRetPose_Rz,quality,errorIndex_P1,errorIndex_P2,errorIndex_P3,errorIndex_P4);
	}
	/**-----------------------Part8 工具坐标与用户坐标读写指令-------------------------*/
	/**
	 * @Title HRIF_SetTCP
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dTCP_X TCP_X
	 * @param dTCP_Y TCP_Y
	 * @param dTCP_Z TCP_Z
	 * @param dTCP_Rx TCP_Rx
	 * @param dTCP_Ry TCP_Ry
	 * @param dTCP_Rz TCP_Rz
	 * @return int,0:调用成功;>0:返回错误码
	 * @Description 设置当前工具坐标 -不写入配置文件，重启后失效
	 */
	public int HRIF_SetTCP(int boxID, int rbtID, double dTCP_X,double dTCP_Y,double dTCP_Z,double dTCP_Rx,double dTCP_Ry,double dTCP_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetTCP_base(rbtID, dTCP_X, dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz);
	}

	/**
	 * @Title HRIF_SetUCS
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dUCS_X UCS_X
	 * @param dUCS_Y UCS_Y
	 * @param dUCS_Z UCS_Z
	 * @param dUCS_Rx UCS_Rx
	 * @param dUCS_Ry UCS_Ry
	 * @param dUCS_Rz UCS_Rz
	 * @return int,0:调用成功;>0:返回错误码
	 * @Description 设置当前用户坐标 -不写入配置文件,重启后失效
	 */
	public int HRIF_SetUCS(int boxID, int rbtID, double dUCS_X,double dUCS_Y,double dUCS_Z,double dUCS_Rx,double dUCS_Ry,double dUCS_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetUCS_base(rbtID, dUCS_X, dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz);
	}

	/**
	 * @Title HRIF_ReadCurTCP
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dTCP_X 工具坐标X
	 * @param dTCP_Y 工具坐标Y
	 * @param dTCP_Z 工具坐标Z
	 * @param dTCP_Rx 工具坐标Rx
	 * @param dTCP_Ry 工具坐标Ry
	 * @param dTCP_Rz 工具坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前设置的工具坐标值
	 */
	public int HRIF_ReadCurTCP(int boxID, int rbtID,  DoubleData dTCP_X, DoubleData dTCP_Y, DoubleData dTCP_Z, DoubleData dTCP_Rx, DoubleData dTCP_Ry, DoubleData dTCP_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCurTCP_base(rbtID, dTCP_X, dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz);
	}

	/**
	 * @Title HRIF_ReadCurUCS
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dUCS_X 用户坐标X
	 * @param dUCS_Y 用户坐标Y
	 * @param dUCS_Z 用户坐标Z
	 * @param dUCS_Rx 用户坐标Rx
	 * @param dUCS_Ry 用户坐标Ry
	 * @param dUCS_Rz 用户坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前设置的用户坐标值
	 */
	public int HRIF_ReadCurUCS(int boxID, int rbtID, DoubleData dUCS_X,DoubleData dUCS_Y,DoubleData dUCS_Z,DoubleData dUCS_Rx,DoubleData dUCS_Ry,DoubleData dUCS_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadCurUCS_base(rbtID, dUCS_X, dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz);
	}

	/**
	 * @Title HRIF_SetTCPByName
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTcpName 工具坐标名称
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 通过名称设置工具坐标列表中的值为当前工具坐标，对应名称为示教器配置页面 TCP示教的工具
					名称
	 */
	public int HRIF_SetTCPByName(int boxID, int rbtID, String sTcpName) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetTCPByName_base(rbtID, sTcpName);
	}

	/**
	 * @Title HRIF_SetUCSByName
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sUcsName 用户坐标名称
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 通过名称设置用户坐标列表中的值为当前用户坐标，对应名称为示教器配置页面用户坐标示教的
					名称
	 */
	public int HRIF_SetUCSByName(int boxID, int rbtID, String sUcsName) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetUCSByName_base(rbtID, sUcsName);
	}

	/**
	 * @Title HRIF_ReadTCPByName
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTcpName 工具坐标名称
	 * @param dTCP_X 工具坐标X
	 * @param dTCP_Y 工具坐标Y
	 * @param dTCP_Z 工具坐标Z
	 * @param dTCP_Rx 工具坐标Rx
	 * @param dTCP_Ry 工具坐标Ry
	 * @param dTCP_Rz 工具坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 通过名称读取指定TCP坐标,对应名称为示教器配置页面 TCP示教的工具名称
	 */
	public int HRIF_ReadTCPByName(int boxID, int rbtID,  String sTcpName, DoubleData dTCP_X, DoubleData dTCP_Y, DoubleData dTCP_Z, DoubleData dTCP_Rx, DoubleData dTCP_Ry, DoubleData dTCP_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadTCPByName_base(rbtID, sTcpName, dTCP_X, dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz);
	}

	/**
	 * @Title HRIF_ReadTCPList
	 * @date 2024年9月23日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取系统中保存的 TCP名称列表
	 */
	public int HRIF_ReadTCPList(int boxID, int rbtID,  StringData TCPList)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadTCPList_base(rbtID,TCPList);
	}

	/**
	 * @Title HRIF_ReadTCPList
	 * @date 2024年9月23日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取系统中保存的 UCS名称列表
	 */
	public int HRIF_ReadUCSList(int boxID, int rbtID,  StringData UCSList)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadUCSList_base(rbtID,UCSList);
	}

	/**
	 * @Title HRIF_ReadUCSByName
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sUcsName 用户坐标名称
	 * @param dUCS_X 用户坐标X
	 * @param dUCS_Y 用户坐标Y
	 * @param dUCS_Z 用户坐标Z
	 * @param dUCS_Rx 用户坐标Rx
	 * @param dUCS_Ry 用户坐标Ry
	 * @param dUCS_Rz 用户坐标Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 通过名称读取指定UCS坐标,对应名称为示教器配置页面用户坐标示教的用户坐标名称
	 */
	public int HRIF_ReadUCSByName(int boxID, int rbtID, String sUcsName, DoubleData dUCS_X,DoubleData dUCS_Y,DoubleData dUCS_Z,DoubleData dUCS_Rx,DoubleData dUCS_Ry,DoubleData dUCS_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadUCSByName_base(rbtID, sUcsName, dUCS_X, dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz);
	}

	/**
	 * @Title HRIF_ConfigTCP
	 * @param boxID    电箱ID
	 * @param rbtID    机器人ID
	 * @param sTcpName
	 * @param dTcp_X
	 * @param dTcp_Y
	 * @param dTcp_Z
	 * @param dTcp_Rx
	 * @param dTcp_Ry
	 * @param dTcp_Rz
	 * @return int, 0：调用成功;>0返回错误码
	 * @date 2024年9月7日
	 * @Description 新建指定名称的TCP和值
	 */
	public int HRIF_ConfigTCP(int boxID, int rbtID, String sTcpName,double dTcp_X,double dTcp_Y,double dTcp_Z,double dTcp_Rx,double dTcp_Ry,double dTcp_Rz) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ConfigTCP_base(rbtID , sTcpName, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz);
	}

	/**
	 * @Title HRIF_ConfigUCS
	 * @param boxID    电箱ID
	 * @param rbtID    机器人ID
	 * @param sTcpName
	 * @param dUcs_X
	 * @param dUcs_Y
	 * @param dUcs_Z
	 * @param dUcs_Rx
	 * @param dUcs_Ry
	 * @param dUcs_Rz
	 * @return int, 0：调用成功;>0返回错误码
	 * @date 2024年9月7日
	 * @Description 新建指定名称的TCP和值
	 */
	public int HRIF_ConfigUCS(int boxID, int rbtID, String sTcpName, double dUcs_X, double dUcs_Y, double dUcs_Z, double dUcs_Rx, double dUcs_Ry, double dUcs_Rz) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ConfigUCS_base(rbtID , sTcpName, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
	}

	/**
	 * @Title HRIF_SetBaseInstallingAngle
	 * @param boxID    电箱ID
	 * @param rbtID    机器人ID
	 * @param nRotation 设定的机座旋转角度
	 * @param nTilt     设定的机座倾斜角度
	 * @return int, 0：调用成功;>0返回错误码
	 * @date 2025年3月20日
	 * @Description 设定机座安装角度
	 */
	public int HRIF_SetBaseInstallingAngle(int boxID, int rbtID, int nRotation, int nTilt) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetBaseInstallingAngle_base(rbtID , nRotation, nTilt);
	}

	/**
	 * @Title HRIF_GetBaseInstallingAngle
	 * @param boxID    电箱ID
	 * @param rbtID    机器人ID
	 * @param nRotation 当前机座旋转角度
	 * @param nTilt     当前机座倾斜角度
	 * @return int, 0：调用成功;>0返回错误码
	 * @date 2025年3月20日
	 * @Description 读取机座安装角度
	 */
	public int HRIF_GetBaseInstallingAngle(int boxID, int rbtID, IntData nRotation, IntData nTilt) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GetBaseInstallingAngle_base(rbtID , nRotation, nTilt);
	}

	/**----------------------- Part9 力控控制指令-------------------------*/

	/**
	 * @Title HRIF_SetForceControlState
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nState 力控状态
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控状态
	 */
	public int HRIF_SetForceControlState(int boxID, int rbtID, int nState) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetForceControlState_base(rbtID, nState);
	}

	/**
	 * @Title HRIF_ReadForceControlState
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nState 力控状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前力控状态
	 */
	public int HRIF_ReadForceControlState(int boxID, int rbtID, IntData nState) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadForceControlState_base(rbtID, nState);
	}

	/**
	 * @Title HRIF_SetForceToolCoordinateMotion
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nMode 模式;0：关闭;1：开启
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控坐标系方向为 tool坐标方向模式
	 */
	public int HRIF_SetForceToolCoordinateMotion(int boxID, int rbtID, int nMode) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetForceToolCoordinateMotion_base(rbtID, nMode);
	}

	/**
	 * @Title HRIF_ForceControlInterrupt
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 暂停力控运动，仅暂停力控功能，不暂停运动和脚本
	 */
	public int HRIF_ForceControlInterrupt(int boxID, int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ForceControlInterrupt_base(rbtID);
	}

	/**
	 * @Title HRIF_ForceControlContinue
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 继续力控运动，仅继续力控运动功能，不继续运动和脚本
	 */
	public int HRIF_ForceControlContinue(int boxID, int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ForceControlContinue_base(rbtID);
	}

	/**
	 * @Title HRIF_SetForceZero
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int  0：调用成功;>0返回错误码
	 * @Description 力控清零,在原有数据的基础上重新标定力传感器
	 */
	public int HRIF_SetForceZero(int boxID, int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetForceZero_base(rbtID);
	}

	/**
	 * @Title HRIF_SetMaxSearchVelocities
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dMaxLinearVelocity 直线速度
	 * @param dMaxAngularVelocity 姿态角速度
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控探寻的最大速度
	 */
	public int HRIF_SetMaxSearchVelocities(int boxID, int rbtID, double dMaxLinearVelocity, double dMaxAngularVelocity) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetMaxSearchVelocities_base(rbtID, dMaxLinearVelocity, dMaxAngularVelocity);
	}

	/**
	 * @Title HRIF_SetControlFreedom
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nX 1:开启 /0关闭
	 * @param nY 1:开启 /0关闭
	 * @param nZ 1:开启 /0关闭
	 * @param nRx 1:开启 /0关闭
	 * @param nRy 1:开启 /0关闭
	 * @param nRz 1:开启 /0关闭
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控探寻自由度
	 */
	public int HRIF_SetControlFreedom(int boxID, int rbtID, int nX, int nY, int nZ, int nRx, int nRy, int  nRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetControlFreedom_base(rbtID, nX, nY, nZ, nRx, nRy, nRz);
	}

	/**
	 * @Title HRIF_SetForceControlStrategy
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nState 控制策略;各轴探寻自由度开关：0：柔顺模式/1：探寻模式
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控控制策略
	 */
	public int HRIF_SetForceControlStrategy(int boxID, int rbtID, int nState) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetForceControlStrategy_base(rbtID, nState);
	}

	/**
	 * @Title HRIF_SetFreeDrivePositionAndOrientation
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX X
	 * @param dY Y
	 * @param dZ Z
	 * @param dRx Rx
	 * @param dRy Ry
	 * @param dRz Rz
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力传感器中心相对于法兰盘的安装位置和姿态
	 */
	public int HRIF_SetFreeDrivePositionAndOrientation(int boxID, int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double  dRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetFreeDrivePositionAndOrientation_base(rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_SetPIDControlParams
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dFp PID参数fP
	 * @param dFi PID参数fI
	 * @param dFd PID参数fD
	 * @param dTp PID参数tP
	 * @param dTi PID参数tI
	 * @param dTd PID参数tD
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控探寻PID参数
	 */
	public int HRIF_SetPIDControlParams(int boxID,  int rbtID, double dFp, double dFi, double dFd, double dTp, double dTi, double dTd)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetPIDControlParams_base(rbtID, dFp, dFi, dFd, dTp, dTi, dTd);
	}

	/**
	 * @Title HRIF_SetMassParams
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX X方向惯量控制参数
	 * @param dY Y方向惯量控制参数
	 * @param dZ Z方向惯量控制参数
	 * @param dRx RX方向惯量控制参数
	 * @param dRy RY方向惯量控制参数
	 * @param dRz RZ方向惯量控制参数
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置惯量控制参数
	 */
	public int HRIF_SetMassParams(int boxID, int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double  dRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetMassParams_base(rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_SetDampParams
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX 阻尼控制参数X方向
	 * @param dY 阻尼控制参数Y方向
	 * @param dZ 阻尼控制参数Z方向
	 * @param dRx 阻尼控制参数RX方向
	 * @param dRy 阻尼控制参数RY方向
	 * @param dRz 阻尼控制参数RZ方向
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置阻尼(b)控制参数
	 */
	public int HRIF_SetDampParams(int boxID, int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double  dRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetDampParams_base(rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_SetStiffParams
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX 刚度控制参数X方向
	 * @param dY 刚度控制参数Y方向
	 * @param dZ 刚度控制参数Z方向
	 * @param dRx 刚度控制参数RX方向
	 * @param dRy 刚度控制参数RY方向
	 * @param dRz 刚度控制参数RZ方向
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置刚度(k)控制参数
	 */
	public int HRIF_SetStiffParams(int boxID, int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double  dRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetStiffParams_base(rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_SetForceControlGoal
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX 力控目标力X方向
	 * @param dY 力控目标力Y方向
	 * @param dZ 力控目标力Z方向
	 * @param dRx 力控目标力RX方向
	 * @param dRy 力控目标力RY方向
	 * @param dRz 力控目标力RZ方向
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控目标力
	 */
	public int HRIF_SetForceControlGoal(int boxID, int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double  dRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetForceControlGoal_base(rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_SetControlGoal
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
 	 * @param dWrench_X X方向力控目标力
	 * @param dWrench_Y Y方向力控目标力
	 * @param dWrench_Z Z方向力控目标力
	 * @param dWrench_Rx Rx方向力控目标力
	 * @param dWrench_Ry Ry方向力控目标力
	 * @param dWrench_Rz Rz方向力控目标力
	 * @param dDistance_X X方向力控目标距离，暂未启用
	 * @param dDistance_Y Y方向力控目标距离，暂未启用
	 * @param dDistance_Z Z方向力控目标距离，暂未启用
	 * @param dDistance_Rx Rx方向力控目标距离，暂未启用
	 * @param dDistance_Ry Ry方向力控目标距离，暂未启用
	 * @param dDistance_Rz Rz方向力控目标距离，暂未启用
	 * @return int, 0:调用成功;>0返回错误码
	 * @Description 设置力控目标力和目标距离
	 */
	public int HRIF_SetControlGoal(int boxID, int rbtID, double dWrench_X, double dWrench_Y, double dWrench_Z, double dWrench_Rx, double dWrench_Ry, double dWrench_Rz,
			double dDistance_X,double dDistance_Y,double dDistance_Z,double dDistance_Rx,double dDistance_Ry,double dDistance_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetControlGoal_base(rbtID, dWrench_X, dWrench_Y, dWrench_Z, dWrench_Rx, dWrench_Ry, dWrench_Rz,
			 dDistance_X, dDistance_Y, dDistance_Z, dDistance_Rx, dDistance_Ry, dDistance_Rz);
	}

	/**
	 * @Title HRIF_SetForceDataLimit
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
 	 * @param dMax_X X方向力最大范围
	 * @param dMax_Y Y方向力最大范围
	 * @param dMax_Z Z方向力最大范围
	 * @param dMax_Rx Rx方向力最大范围
	 * @param dMax_Ry Ry方向力最大范围
	 * @param dMax_Rz Rz方向力最大范围
	 * @param dMin_X X方向力最小范围
	 * @param dMin_Y Y方向力最小范围
	 * @param dMin_Z Z方向力最小范围
	 * @param dMin_Rx Rx方向力最小范围
	 * @param dMin_Ry Ry方向力最小范围
	 * @param dMin_Rz Rz方向力最小范围
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控限制范围 -力传感器超过此范围后控制器断电
	 */
	public int HRIF_SetForceDataLimit(int boxID, int rbtID, double dMax_X, double dMax_Y, double dMax_Z, double dMax_Rx, double dMax_Ry, double dMax_Rz,
			double dMin_X,double dMin_Y,double dMin_Z,double dMin_Rx,double dMin_Ry,double dMin_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetForceDataLimit_base(rbtID, dMax_X, dMax_Y, dMax_Z, dMax_Rx, dMax_Ry, dMax_Rz,
				 dMin_X, dMin_Y, dMin_Z, dMin_Rx, dMin_Ry, dMin_Rz);
	}

	/**
	 * @Title HRIF_SetForceDistanceLimit
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dAllowDistance 允许最大距离
	 * @param dStrengthLevel 位置与边界设置偏离距离的幂次项;2：阻力与偏离边界的平方项成比例；设成3：就表示阻力与偏离边界的立方项成比例
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控形变范围
	 */
	public int HRIF_SetForceDistanceLimit(int boxID, int rbtID, double dAllowDistance, double dStrengthLevel)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetForceDistanceLimit_base(rbtID, dAllowDistance, dStrengthLevel);
	}

	/**
	 * @Title HRIF_SetForceFreeDriveMode
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param bEnable 是否开启,1:开启 /0:关闭
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置开启或者关闭力控自由驱动模式
	 */
	public int HRIF_SetForceFreeDriveMode(int boxID, int rbtID, int bEnable) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetForceFreeDriveMode_base(rbtID, bEnable);
	}

	/**
	 * @Title HRIF_ReadFTCabData
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX X方向标定后的力传感器数据
	 * @param dY Y方向标定后的力传感器数据
	 * @param dZ Z方向标定后的力传感器数据
	 * @param dRx Rx方向标定后的力传感器数据
	 * @param dRy Ry方向标定后的力传感器数据
	 * @param dRz Rz方向标定后的力传感器数据
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取标定后的力传感器数据
	 */
	public int  HRIF_ReadFTCabData(int boxID, int rbtID, DoubleData dX,DoubleData dY,DoubleData dZ,DoubleData dRx,DoubleData dRy,DoubleData dRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadFTCabData_base(rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_ReadFTFreeDriveSpeedMode
	 * @date 2024年9月7日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nMode 速度模式
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取标定后的力传感器数据
	 */
	public int  HRIF_ReadFTFreeDriveSpeedMode(int boxID, int rbtID,IntData nMode)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadFTFreeDriveSpeedMode_base(rbtID,nMode);
	}

	/**
	 * @Title HRIF_SetFTFreeDriveSpeedMode
	 * @date 2024年9月7日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nMode 速度模式
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置力控自由驱动速度模式
	 */
	public int  HRIF_SetFTFreeDriveSpeedMode(int boxID, int rbtID,int nMode)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetFTFreeDriveSpeedMode_base(rbtID,nMode);
	}

	/**
	 * @Title HRIF_SetFreeDriveMotionFreedom
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nX 各方向自由度
	 * @param nY
	 * @param nZ
	 * @param nRx
	 * @param nRy
	 * @param nRz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @date 2024年9月9日
	 * @Description 设置力控自由驱动末端自由度。
	 */
	public int  HRIF_SetFreeDriveMotionFreedom(int boxID, int rbtID,int nX,int nY,int nZ,int nRx,int nRy,int nRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetFreeDriveMotionFreedom_base(rbtID,nX,nY,nZ,nRx,nRy,nRz);
	}

	/**
	 * @Title HRIF_ReadFTMotionFreedom
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nX 各方向自由度
	 * @param nY
	 * @param nZ
	 * @param nRx
	 * @param nRy
	 * @param nRz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @date 2024年9月23日
	 * @Description 读取力控自由驱动末端自由度。
	 */
	public int  HRIF_ReadFTMotionFreedom(int boxID, int rbtID,IntData nX,IntData nY,IntData nZ,IntData nRx,IntData nRy,IntData nRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadFTMotionFreedom_base(rbtID,nX,nY,nZ,nRx,nRy,nRz);
	}

	/**
	 * @Title HRIF_SetFTFreeFactor
	 * @date 2024年9月7日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dLinear 平移柔顺度
	 * @param dAngular 旋转柔顺度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取标定后的力传感器数据dLinear
	 */
	public int  HRIF_SetFTFreeFactor(int boxID, int rbtID,double dLinear,double dAngular)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetFTFreeFactor_base(rbtID,dLinear,dAngular);
	}

	/**
	 * @Title HRIF_SetFreeDriveCompensateForce
	 * @date 2024年9月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dForce 补偿力
	 * @param dX 补偿力的方向
	 * @param dY
	 * @param dZ
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置FreeDrive模式下的定向补偿力大小及矢量方向
	 */
	public int  HRIF_SetFreeDriveCompensateForce(int boxID, int rbtID,double dForce,double dX ,double dY,double dZ)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetFreeDriveCompensateForce_base(rbtID,dForce,dX,dY,dZ);
	}

	/**
	 * @Title HRIF_SetFTWrenchThresholds
	 * @date 2024年9月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dForceThreshold 补偿力的方向
	 * @param dTorqueThreshold
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置力控自由驱动启动阈值（力与力矩）
	 */
	public int  HRIF_SetFTWrenchThresholds(int boxID, int rbtID,double dForceThreshold,double dTorqueThreshold )
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetFTWrenchThresholds_base(rbtID,dForceThreshold,dTorqueThreshold);
	}

	/**
	 * @Title HRIF_SetMaxFreeDriveVel
	 * @date 2024年9月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dMaxLinearVelocity 直线速度
	 * @param dMaxAngularVelocity 姿态角速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置力控自由驱动最大直线速度及姿态角速度
	 */
	public int  HRIF_SetMaxFreeDriveVel(int boxID, int rbtID,double dMaxLinearVelocity,double dMaxAngularVelocity )
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetMaxFreeDriveVel_base(rbtID,dMaxLinearVelocity,dMaxAngularVelocity);
	}

	/**
	 * @param boxID               电箱ID
	 * @param rbtID               机器人ID
	 * @param Dis_X
	 * @param Dis_Y
	 * @param Dis_Z
	 * @param Dis_RX
	 * @param Dis_RY
	 * @param Dis_RZ
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Title HRIF_SetMaxSearchDistance
	 * @date 2024年9月10日
	 * @Description 设置各自由度力控探寻最大距离
	 */
	public int  HRIF_SetMaxSearchDistance(int boxID, int rbtID,double Dis_X,double Dis_Y,double Dis_Z,double Dis_RX,double Dis_RY,double Dis_RZ )
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetMaxSearchDistance_base(rbtID,Dis_X,Dis_Y,Dis_Z,Dis_RX,Dis_RY,Dis_RZ);
	}

	/**
	 * @param boxID               电箱ID
	 * @param rbtID               机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Title HRIF_SetSteadyContactDeviationRange
	 * @date 2024年9月10日
	 * @Description 设置恒力控稳定阶段边界
	 */
	public int  HRIF_SetSteadyContactDeviationRange(int boxID,int rbtID,double Pos_X,double Pos_Y,double Pos_Z,double Pos_RX,double Pos_RY,double Pos_RZ,
													double Neg_X,double Neg_Y,double Neg_Z,double Neg_RX,double Neg_RY,double Neg_RZ )
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetSteadyContactDeviationRange_base(rbtID,Pos_X,Pos_Y,Pos_Z,Pos_RX,Pos_RY,Pos_RZ,
				Neg_X,Neg_Y,Neg_Z,Neg_RX,Neg_RY,Neg_RZ);
	}

	/**
	 * @Title HRIF_SetTangentForceBounds
	 * @date 2024年9月7日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dVel 抬升速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置X/Y方向切向力最大值、最小值和最大上抬速度。
	 */
	public int  HRIF_SetTangentForceBounds(int boxID, int rbtID,double dMax,double dMin,double dVel)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetTangentForceBounds_base(rbtID,dMax,dMin,dVel);
	}


	/**
	 * @Title HRIF_AddSafePlane
	 * @date 2024年9月27日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 添加虚拟墙平面
	 */
	public int  HRIF_AddSafePlane(int boxID, int rbtID,String Name,String UcsName ,int Mode,int Display,int Switch)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_AddSafePlane_base(rbtID,Name, UcsName , Mode, Display, Switch);
	}

	/**
	 * @Title HRIF_UpdateSafePlane
	 * @date 2024年9月27日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 修改更新虚拟墙平面属性
	 */
	public int  HRIF_UpdateSafePlane(int boxID, int rbtID,String Name,String UcsName ,int Mode,int Display,int Switch)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_UpdateSafePlane_base(rbtID,Name, UcsName , Mode, Display, Switch);
	}

	/**
	 * @Title HRIF_DelSafePlane
	 * @date 2024年9月27日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 删除虚拟墙平面
	 */
	public int  HRIF_DelSafePlane(int boxID, int rbtID,String Name)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_DelSafePlane_base(rbtID,Name);
	}

	/**
	 * @Title HRIF_ReadSafePlaneList
	 * @date 2024年9月27日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 返回结果为所有安全平面的名字清单
	 */
	public int  HRIF_ReadSafePlaneList(int boxID, int rbtID,StringData BordersName )
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadSafePlaneList_base(rbtID,BordersName );
	}

	/**
	 * @Title HRIF_ReadSafePlane
	 * @date 2024年9月27日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 返回结果为指定安全平面的详细参数
	 */
	public int  HRIF_ReadSafePlane(int boxID, int rbtID,String BorderName ,StringData UcsName ,IntData Mode,IntData Display,IntData Switch)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadSafePlane_base(rbtID, BorderName, UcsName, Mode, Display, Switch);
	}

	/**
	 * @Title HRIF_SetDepthThresholdForDampingArea
	 * @date 2024年9月27日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置虚拟墙开始产生阻尼时的距离阈值
	 */
	public int  HRIF_SetDepthThresholdForDampingArea(int boxID, int rbtID,double dDepth)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetDepthThresholdForDampingArea_base(rbtID, dDepth);
	}

	/**
	 * @Title HRIF_GetLastCalibParams
	 * @date 2024年11月30日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param Fx0-Fz0 x-z方向标定力
	 * @param Mx0-Mz0 x-z方向力矩
	 * @param X-Z 重心偏移
	 * @param G 重力
	 * @param InstallRotationAngel 机器安装角度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 获取力控标定结果
	 */
	public int  HRIF_GetLastCalibParams(int boxID, int rbtID,DoubleData Fx0,DoubleData Fy0,DoubleData Fz0,DoubleData Mx0,DoubleData My0,DoubleData Mz0,DoubleData G,DoubleData X,DoubleData Y,DoubleData Z,DoubleData InstallRotationAngel)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GetLastCalibParams_base(rbtID,Fx0,Fy0,Fz0,Mx0,My0,Mz0,G,X,Y,Z,InstallRotationAngel);
	}


	/**
	 * @Title HRIF_SetInitializeForceSensor
	 * @date 2024年11月30日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param Fx0-Fz0 x-z方向标定力
	 * @param Mx0-Mz0 x-z方向力矩
	 * @param X-Z 重心偏移
	 * @param G 重力
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置力控标定结果
	 */
	public int  HRIF_SetInitializeForceSensor(int boxID, int rbtID,double Fx0,double Fy0,double Fz0,double Mx0,double My0,double Mz0,double G,double X,double Y,double Z,double InstallRotationAngel)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetInitializeForceSensor_base(rbtID,Fx0,Fy0,Fz0,Mx0,My0,Mz0,G,X,Y,Z,InstallRotationAngel);
	}


	/**
	 * @Title HRIF_SetFTCalibration
	 * @date 2025年3月20日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param Pointnum 标定点位数量
	 * @param Points 标定点位置信息
	 * @param forces 标定点力信息
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 计算力控标定数据，接收 8~16 组标定数据，每组数据包含 12 个参数（6 个位置参数 + 6 个力控参数）
	 */
	public int  HRIF_SetFTCalibration(int boxID, int rbtID,int Pointnum,Vector<Position> Points,Vector<Position> forces)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetFTCalibration_base(rbtID,Pointnum,Points,forces);
	}

	/**
	 * @Title HRIF_ReadForceData
	 * @date 2025年3月20日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dx-dRz x-Rz方向原始力数据
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取力传感器原始数据
	 */
	public int  HRIF_ReadForceData(int boxID, int rbtID,DoubleData dx,DoubleData dy,DoubleData dz,DoubleData dRx,DoubleData dRy,DoubleData dRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadForceData_base(rbtID,dx,dy,dz,dRx,dRy,dRz);
	}

	/**----------------------- Part10 通用运动类控制指令-------------------------*/

	/**
	 * @Title HRIF_ShortJogJ
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param naxisId 关节轴ID
	 * @param nderection 运动方向
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 关节短点动 运动距离2度,最大速度10度/s
	 */
	public int HRIF_ShortJogJ(int boxID,int rbtID, int naxisId, int nderection) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ShortJogJ_base(rbtID, naxisId, nderection);
	}

	/**
	 * @Title HRIF_ShortJogL
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param naxisId 坐标系轴ID
	 * @param nderection 方向
	 * @return int, 0:调用成功;>0返回错误码
	 * @Description 空间坐标短点动 运动距离2mm，最大速度10mm/s
	 */
	public int HRIF_ShortJogL(int boxID,int rbtID, int naxisId, int nderection) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ShortJogL_base(rbtID, naxisId, nderection);
	}

	/**
	 * @Title HRIF_LongJogJ
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param naxisId 轴ID
	 * @param nderection 方向
	 * @param nstate 状态
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 关节长点动，最大运动速度10/s
	 * @Attention 必须要与HRIF_LongMoveEvent()配合使用
	 */
	public int HRIF_LongJogJ(int boxID,int rbtID, int naxisId, int nderection, int nstate) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_LongJogJ_base(rbtID, naxisId, nderection, nstate);
	}

	/**
	 * @Title HRIF_LongJogL
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param naxisId 轴ID
	 * @param nderection 方向
	 * @param nstate 状态
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 空间长点动,最大运动速度50mm/s
	 * @Attention 必须要与HRIF_LongMoveEvent()配合使用
	 */
	public int HRIF_LongJogL(int boxID,int rbtID, int naxisId, int nderection, int nstate) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_LongJogL_base(rbtID, naxisId, nderection, nstate);
	}

	/**
	 * @Title HRIF_LongMoveEvent
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 长点动继续指令，当开始长点动之后，要按 500 毫秒或更短时间为时间周期发送一次该指令，否
则长点动会停止
	 */
	public int HRIF_LongMoveEvent(int boxID, int rbtID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_LongMoveEvent_base(rbtID);
	}

	/**
	 * @Title HRIF_IsMotionDone
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param bDone 是否处于运动状态
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 判断机器人是否处于运动状态
	 */
	public int HRIF_IsMotionDone(int boxID, int rbtID, BooleanData bDone)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_IsMotionDone_base(rbtID, bDone);
	}

	/**
	 * @Title HRIF_IsBlendingDone
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param bDone 路点运动是否完成
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 判断路点是否运动完成
	 */
	public int HRIF_IsBlendingDone(int boxID, int rbtID, BooleanData bDone)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_IsBlendingDone_base(rbtID, bDone);
	}

	/**
	 * @Title HRIF_WayPointEx
	 * @date 2022年6月9日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param moveType 运动类型
	 * @param dX 迪卡尔坐标X
	 * @param dY 迪卡尔坐标Y
	 * @param dZ 迪卡尔坐标Z
	 * @param dRx 迪卡尔坐标Rx
	 * @param dRy 迪卡尔坐标Ry
	 * @param dRz 迪卡尔坐标Rz
	 * @param dJ1 J1关节坐标
	 * @param dJ2 J2关节坐标
	 * @param dJ3 J3关节坐标
	 * @param dJ4 J4关节坐标
	 * @param dJ5 J5关节坐标
	 * @param dJ6 J6关节坐标
	 * @param dTCP_X 工具坐标X
	 * @param dTCP_Y 工具坐标Y
	 * @param dTCP_Z 工具坐标Z
	 * @param dTCP_Rx 工具坐标Rx
	 * @param dTCP_Ry 工具坐标Ry
	 * @param dTCP_Rz 工具坐标Rz
	 * @param dUCS_X 用户坐标X
	 * @param dUCS_Y 用户坐标Y
	 * @param dUCS_Z 用户坐标Z
	 * @param dUCS_Rx 用户坐标Rx
	 * @param dUCS_Ry 用户坐标Ry
	 * @param dUCS_Rz 用户坐标Rz
	 * @param dvelocity 速度
	 * @param dacceleration 加速度
	 * @param dradius 过渡半径
	 * @param nisUseJoint 是否使用关节坐标
	 * @param nisSeek 是否检测DI停止
	 * @param nIOBit 检测的DI索引
	 * @param nIOState 检测的DI状态
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 路点运动
 	 * @Attention HRIF_WayPointEx与HRIF_WayPoint区别在于HRIF_WayPointEx需要设置工具坐标与用户坐标具体的值，而HRIF_WayPoint使用示教器示教的对应工具坐标与用户坐标名称;建议使用HRIF_MoveJ, HRIF_MoveL, HRIF_MoveC
	 */
	public int HRIF_WayPointEx(int boxID, int rbtID, int moveType,
			double dX, double dY, double dZ, double dRx, double dRy, double dRz,
			double dJ1, double dJ2,double dJ3,double dJ4,double dJ5,double dJ6,
			double dTCP_X, double dTCP_Y,double dTCP_Z,double dTCP_Rx,double dTCP_Ry,double dTCP_Rz,
			double dUCS_X, double dUCS_Y,double dUCS_Z,double dUCS_Rx,double dUCS_Ry,double dUCS_Rz,
			double dvelocity, double dacceleration, double dradius, int nisUseJoint, int nisSeek, int nIOBit, int nIOState, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_WayPointEx_base(rbtID, moveType,
			 dX, dY, dZ, dRx, dRy, dRz,
			 dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
			 dTCP_X, dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz,
			 dUCS_X, dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz,
			 dvelocity, dacceleration, dradius, nisUseJoint, nisSeek, nIOBit, nIOState, strCmdID);
	}

	/**
	 * @Title HRIF_WayPointEx_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param moveType 运动类型
	 * @param dX 迪卡尔坐标X
	 * @param dY 迪卡尔坐标Y
	 * @param dZ 迪卡尔坐标Z
	 * @param dRx 迪卡尔坐标Rx
	 * @param dRy 迪卡尔坐标Ry
	 * @param dRz 迪卡尔坐标Rz
	 * @param dTCP_X 工具坐标X
	 * @param dTCP_Y 工具坐标Y
	 * @param dTCP_Z 工具坐标Z
	 * @param dTCP_Rx 工具坐标Rx
	 * @param dTCP_Ry 工具坐标Ry
	 * @param dTCP_Rz 工具坐标Rz
	 * @param dUCS_X 用户坐标X
	 * @param dUCS_Y 用户坐标Y
	 * @param dUCS_Z 用户坐标Z
	 * @param dUCS_Rx 用户坐标Rx
	 * @param dUCS_Ry 用户坐标Ry
	 * @param dUCS_Rz 用户坐标Rz
	 * @param dvelocity 速度
	 * @param dacceleration 加速度
	 * @param dradius 过渡半径
	 * @param nisUseJoint 是否使用关节坐标
	 * @param nisSeek 是否检测DI停止
	 * @param nIOBit 检测的DI索引
	 * @param nIOState 检测的DI状态
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 路点运动
	 * @Attention HRIF_WayPointEx与HRIF_WayPoint区别在于HRIF_WayPointEx需要设置工具坐标与用户坐标具体的值，而HRIF_WayPoint使用示教器示教的对应工具坐标与用户坐标名称;建议使用HRIF_MoveJ, HRIF_MoveL, HRIF_MoveC
	 */
	public int HRIF_WayPointEx_nj(int boxID, int rbtID, int moveType,
							   double dX, double dY, double dZ, double dRx, double dRy, double dRz,
								  JointsDatas joints,
							   double dTCP_X, double dTCP_Y,double dTCP_Z,double dTCP_Rx,double dTCP_Ry,double dTCP_Rz,
							   double dUCS_X, double dUCS_Y,double dUCS_Z,double dUCS_Rx,double dUCS_Ry,double dUCS_Rz,
							   double dvelocity, double dacceleration, double dradius, int nisUseJoint, int nisSeek, int nIOBit, int nIOState, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_WayPointEx_nj_base(rbtID, moveType,
				dX, dY, dZ, dRx, dRy, dRz,
				 joints,
				dTCP_X, dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz,
				dUCS_X, dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz,
				dvelocity, dacceleration, dradius, nisUseJoint, nisSeek, nIOBit, nIOState, strCmdID);
	}

	/**
	 * @Title HRIF_WayPoint
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nmoveType 运动类型,0：关节运动;1：直线运动
	 * @param dX 迪卡尔坐标X
	 * @param dY 迪卡尔坐标Y
	 * @param dZ 迪卡尔坐标Z
	 * @param dRx 迪卡尔坐标Rx
	 * @param dRy 迪卡尔坐标Ry
	 * @param dRz 迪卡尔坐标Rz
	 * @param dJ1 J1关节坐标
	 * @param dJ2 J2关节坐标
	 * @param dJ3 J3关节坐标
	 * @param dJ4 J4关节坐标
	 * @param dJ5 J5关节坐标
	 * @param dJ6 J6关节坐标
	 * @param stcpName TCP的名称
	 * @param sucsName UCS的名称
	 * @param dvelocity 运动最大速度，关节运动时单位[°/s]，空间运动时XYZ单位[mm/s]，Rx，Ry，Rz单位[°/s]
	 * @param dacceleration 运动最大加速度，关节运动时单位[°/s2]，空间运动时XYZ单位[mm/s2]，Rx，Ry，Rz单位[°/s2]
	 * @param dradius 过渡半径，单位[mm]
	 * @param nisUseJoint 是否使用关节坐标,是否使用关节角度作为目标点，如果nMoveType=0时，则nisJoint有效：
			   0：不使用关节角度;1：使用关节角度
	 * @param nisSeek 是否检测DI停止,如果nIsSeek为1，则开启检测DI停止，路点运动过程中如果电箱的nIOBit位索引的DI的状态=nIOState时，机器人停止运动，否则运动到目标点完成运动
	 * @param nbit 检测的DI索引,电箱对应DI索引，nIsSeek=0时无效
	 * @param nstate 检测的DI状态,检测的DI状态，，nIsSeek=0时无效
	 * @param strCmdID 路点全局变量，可以由客户设置，是数字与字母的字符串，不要出现 ,;两种符号，否则会解释指令失败
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 路点运动
	 * @Attention HRIF_WayPointEx与HRIF_WayPoint区别在于HRIF_WayPointEx需要设置工具坐标与用户坐标具体的值，而HRIF_WayPoint使用示教器示教的对应工具坐标与用户坐标名称;建议使用HRIF_MoveJ, HRIF_MoveL, HRIF_MoveC
	 */
	public int HRIF_WayPoint(int boxID, int rbtID, int nmoveType,
			double dX, double dY, double dZ, double dRx, double dRy, double dRz,
			double dJ1, double dJ2,double dJ3,double dJ4,double dJ5,double dJ6,
			String stcpName, String sucsName,
			double dvelocity, double dacceleration, double dradius, int nisUseJoint, int nisSeek, int nbit,
			int nstate, String strCmdID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_WayPoint_base(rbtID, nmoveType,
				 dX, dY, dZ, dRx, dRy, dRz,
				 dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
				 stcpName, sucsName,
				 dvelocity, dacceleration, dradius, nisUseJoint, nisSeek, nbit,
				 nstate, strCmdID);
	}

	/**
	 * @Title HRIF_WayPoint_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nmoveType 运动类型,0：关节运动;1：直线运动
	 * @param dX 迪卡尔坐标X
	 * @param dY 迪卡尔坐标Y
	 * @param dZ 迪卡尔坐标Z
	 * @param dRx 迪卡尔坐标Rx
	 * @param dRy 迪卡尔坐标Ry
	 * @param dRz 迪卡尔坐标Rz
	 * @param stcpName TCP的名称
	 * @param sucsName UCS的名称
	 * @param dvelocity 运动最大速度，关节运动时单位[°/s]，空间运动时XYZ单位[mm/s]，Rx，Ry，Rz单位[°/s]
	 * @param dacceleration 运动最大加速度，关节运动时单位[°/s2]，空间运动时XYZ单位[mm/s2]，Rx，Ry，Rz单位[°/s2]
	 * @param dradius 过渡半径，单位[mm]
	 * @param nisUseJoint 是否使用关节坐标,是否使用关节角度作为目标点，如果nMoveType=0时，则nisJoint有效：
	0：不使用关节角度;1：使用关节角度
	 * @param nisSeek 是否检测DI停止,如果nIsSeek为1，则开启检测DI停止，路点运动过程中如果电箱的nIOBit位索引的DI的状态=nIOState时，机器人停止运动，否则运动到目标点完成运动
	 * @param nbit 检测的DI索引,电箱对应DI索引，nIsSeek=0时无效
	 * @param nstate 检测的DI状态,检测的DI状态，，nIsSeek=0时无效
	 * @param strCmdID 路点全局变量，可以由客户设置，是数字与字母的字符串，不要出现 ,;两种符号，否则会解释指令失败
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 路点运动
	 * @Attention HRIF_WayPointEx与HRIF_WayPoint区别在于HRIF_WayPointEx需要设置工具坐标与用户坐标具体的值，而HRIF_WayPoint使用示教器示教的对应工具坐标与用户坐标名称;建议使用HRIF_MoveJ, HRIF_MoveL, HRIF_MoveC
	 */
	public int HRIF_WayPoint_nj(int boxID, int rbtID, int nmoveType,
							 double dX, double dY, double dZ, double dRx, double dRy, double dRz,
								JointsDatas joints,
							 String stcpName, String sucsName,
							 double dvelocity, double dacceleration, double dradius, int nisUseJoint, int nisSeek, int nbit,
							 int nstate, String strCmdID) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_WayPoint_nj_base(rbtID, nmoveType,
				dX, dY, dZ, dRx, dRy, dRz,
				 joints,
				stcpName, sucsName,
				dvelocity, dacceleration, dradius, nisUseJoint, nisSeek, nbit,
				nstate, strCmdID);
	}

	/**
	 * @Title HRIF_WayPoint2
	 * @date 2022年6月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nmoveType 运动类型
 	 * @param dEndPose_X 空间目标位置X
	 * @param dEndPose_Y 空间目标位置Y
	 * @param dEndPose_Z 空间目标位置Z
	 * @param dEndPose_Rx 空间目标位置Rx
	 * @param dEndPose_Ry 空间目标位置Ry
	 * @param dEndPose_Rz 空间目标位置Rz
	 * @param dAuxPose_X 空间目标位置X
	 * @param dAuxPose_Y 空间目标位置Y
	 * @param dAuxPose_Z 空间目标位置Z
	 * @param dAuxPose_Rx 空间目标位置Rx
	 * @param dAuxPose_Ry 空间目标位置Ry
	 * @param dAuxPose_Rz 空间目标位置Rz
	 * @param dJ1 关节目标位置J1
	 * @param dJ2 关节目标位置J2
	 * @param dJ3 关节目标位置J3
	 * @param dJ4 关节目标位置J4
	 * @param dJ5 关节目标位置J5
	 * @param dJ6 关节目标位置J6
	 * @param sTcpName 工具坐标名称
	 * @param sUcsName 用户坐标名称
	 * @param dvelocity 速度
	 * @param dacceleration 加速度
	 * @param dradius 过渡半径
	 * @param nisUseJoint 是否使用关节坐标
	 * @param nisSeek 是否检测DI停止
	 * @param nIOBit 检测的DI索引
	 * @param nIOState 检测的DI状态
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 路点运动
	 */
	public int HRIF_WayPoint2(int boxID, int rbtID, int nmoveType,
			double dEndPose_X, double dEndPose_Y, double dEndPose_Z, double dEndPose_Rx, double dEndPose_Ry, double dEndPose_Rz,
			double dAuxPose_X, double dAuxPose_Y, double dAuxPose_Z, double dAuxPose_Rx, double dAuxPose_Ry, double dAuxPose_Rz,
			double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6,
			String sTcpName, String sUcsName,
			double dvelocity, double dacceleration, double dradius, int nisUseJoint, int nisSeek, int nIOBit, int nIOState, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_WayPoint2_base(rbtID, nmoveType,
				 dEndPose_X, dEndPose_Y, dEndPose_Z, dEndPose_Rx, dEndPose_Ry, dEndPose_Rz,
				 dAuxPose_X, dAuxPose_Y,  dAuxPose_Z, dAuxPose_Rx, dAuxPose_Ry, dAuxPose_Rz,
				 dJ1, dJ2, dJ3,  dJ4, dJ5, dJ6,
				 sTcpName, sUcsName,
				 dvelocity, dacceleration, dradius, nisUseJoint, nisSeek, nIOBit, nIOState, strCmdID);
	}

	/**
	 * @Title HRIF_WayPoint2_nj
	 * @date 2022年6月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nmoveType 运动类型
	 * @param dEndPose_X 空间目标位置X
	 * @param dEndPose_Y 空间目标位置Y
	 * @param dEndPose_Z 空间目标位置Z
	 * @param dEndPose_Rx 空间目标位置Rx
	 * @param dEndPose_Ry 空间目标位置Ry
	 * @param dEndPose_Rz 空间目标位置Rz
	 * @param dAuxPose_X 空间目标位置X
	 * @param dAuxPose_Y 空间目标位置Y
	 * @param dAuxPose_Z 空间目标位置Z
	 * @param dAuxPose_Rx 空间目标位置Rx
	 * @param dAuxPose_Ry 空间目标位置Ry
	 * @param dAuxPose_Rz 空间目标位置Rz
	 * @param sTcpName 工具坐标名称
	 * @param sUcsName 用户坐标名称
	 * @param dvelocity 速度
	 * @param dacceleration 加速度
	 * @param dradius 过渡半径
	 * @param nisUseJoint 是否使用关节坐标
	 * @param nisSeek 是否检测DI停止
	 * @param nIOBit 检测的DI索引
	 * @param nIOState 检测的DI状态
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 路点运动
	 */
	public int HRIF_WayPoint2_nj(int boxID, int rbtID, int nmoveType,
							  double dEndPose_X, double dEndPose_Y, double dEndPose_Z, double dEndPose_Rx, double dEndPose_Ry, double dEndPose_Rz,
							  double dAuxPose_X, double dAuxPose_Y, double dAuxPose_Z, double dAuxPose_Rx, double dAuxPose_Ry, double dAuxPose_Rz,
								 JointsDatas joints,
							  String sTcpName, String sUcsName,
							  double dvelocity, double dacceleration, double dradius, int nisUseJoint, int nisSeek, int nIOBit, int nIOState, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_WayPoint2_nj_base(rbtID, nmoveType,
				dEndPose_X, dEndPose_Y, dEndPose_Z, dEndPose_Rx, dEndPose_Ry, dEndPose_Rz,
				dAuxPose_X, dAuxPose_Y,  dAuxPose_Z, dAuxPose_Rx, dAuxPose_Ry, dAuxPose_Rz,
				 joints,
				sTcpName, sUcsName,
				dvelocity, dacceleration, dradius, nisUseJoint, nisSeek, nIOBit, nIOState, strCmdID);
	}

	/**
	 * @Title HRIF_MoveJ
	 * @date 2022年6月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX 迪卡尔坐标X
	 * @param dY 迪卡尔坐标Y
	 * @param dZ 迪卡尔坐标Z
	 * @param dRx 迪卡尔坐标Rx
	 * @param dRy 迪卡尔坐标Ry
	 * @param dRz 迪卡尔坐标Rz
	 * @param dJ1 J1关节坐标
	 * @param dJ2 J2关节坐标
	 * @param dJ3 J3关节坐标
	 * @param dJ4 J4关节坐标
	 * @param dJ5 J5关节坐标
	 * @param dJ6 J6关节坐标
	 * @param sTcpName 工具坐标名称
	 * @param sUcsName 用户坐标名称
	 * @param dvelocity 速度
	 * @param dacc 加速度
	 * @param dradius 过渡半径
	 * @param nIsUseJoint 是否使用关节坐标
	 * @param nIsSeek 是否检测DI停止
	 * @param nIOBit 检测的DI索引
	 * @param nIOState 检测的DI状态
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 关节运动
	 */
	public int HRIF_MoveJ(int boxID, int rbtID,
			double dX, double dY, double dZ, double dRx, double dRy, double dRz,
			double dJ1, double dJ2,double dJ3,double dJ4,double dJ5,double dJ6,
			String sTcpName, String sUcsName,
			double dvelocity, double dacc, double dradius, int nIsUseJoint, int nIsSeek, int nIOBit, int nIOState, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveJ_base(rbtID,
				dX, dY, dZ, dRx, dRy, dRz,
				dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
				sTcpName, sUcsName,
				dvelocity, dacc, dradius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
	}

	/**
	 * @Title HRIF_MoveJ_nj
	 * @date 2024年10月12日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX 迪卡尔坐标X
	 * @param dY 迪卡尔坐标Y
	 * @param dZ 迪卡尔坐标Z
	 * @param dRx 迪卡尔坐标Rx
	 * @param dRy 迪卡尔坐标Ry
	 * @param dRz 迪卡尔坐标Rz
	 * @param sTcpName 工具坐标名称
	 * @param sUcsName 用户坐标名称
	 * @param dvelocity 速度
	 * @param dacc 加速度
	 * @param dradius 过渡半径
	 * @param nIsUseJoint 是否使用关节坐标
	 * @param nIsSeek 是否检测DI停止
	 * @param nIOBit 检测的DI索引
	 * @param nIOState 检测的DI状态
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 关节运动
	 */
	public int HRIF_MoveJ_nj(int boxID, int rbtID,
						  double dX, double dY, double dZ, double dRx, double dRy, double dRz,
							 JointsData joints,
						  String sTcpName, String sUcsName,
						  double dvelocity, double dacc, double dradius, int nIsUseJoint, int nIsSeek, int nIOBit, int nIOState, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveJ_nj_base(rbtID,
				dX, dY, dZ, dRx, dRy, dRz,
				joints,
				sTcpName, sUcsName,
				dvelocity, dacc, dradius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
	}

	/**
	 * @Title HRIF_MoveL
	 * @date 2022年6月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX 迪卡尔坐标X
	 * @param dY 迪卡尔坐标Y
	 * @param dZ 迪卡尔坐标Z
	 * @param dRx 迪卡尔坐标Rx
	 * @param dRy 迪卡尔坐标Ry
	 * @param dRz 迪卡尔坐标Rz
	 * @param dJ1 J1关节坐标
	 * @param dJ2 J2关节坐标
	 * @param dJ3 J3关节坐标
	 * @param dJ4 J4关节坐标
	 * @param dJ5 J5关节坐标
	 * @param dJ6 J6关节坐标
	 * @param sTcpName 工具坐标名称
	 * @param sUcsName 用户坐标名称
	 * @param dvelocity 速度
	 * @param dacc 加速度
	 * @param dradius 过渡半径
	 * @param nIsSeek 是否检测DI停止
	 * @param nIOBit 检测的DI索引
	 * @param nIOState 检测的DI状态
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 直线轨迹运动
	 */
	public int HRIF_MoveL(int boxID, int rbtID,
			double dX, double dY, double dZ, double dRx, double dRy, double dRz,
			double dJ1, double dJ2,double dJ3,double dJ4,double dJ5,double dJ6,
			String sTcpName, String sUcsName,
			double dvelocity, double dacc, double dradius, int nIsSeek, int nIOBit, int nIOState, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveL_base(rbtID,
				 dX, dY, dZ, dRx, dRy, dRz,
				 dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
				 sTcpName, sUcsName,
				 dvelocity, dacc, dradius, nIsSeek, nIOBit, nIOState, strCmdID);
	}

	/**
	 * @Title HRIF_MoveL_nj
	 * @date 2022年6月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX 迪卡尔坐标X
	 * @param dY 迪卡尔坐标Y
	 * @param dZ 迪卡尔坐标Z
	 * @param dRx 迪卡尔坐标Rx
	 * @param dRy 迪卡尔坐标Ry
	 * @param dRz 迪卡尔坐标Rz
	 * @param sTcpName 工具坐标名称
	 * @param sUcsName 用户坐标名称
	 * @param dvelocity 速度
	 * @param dacc 加速度
	 * @param dradius 过渡半径
	 * @param nIsSeek 是否检测DI停止
	 * @param nIOBit 检测的DI索引
	 * @param nIOState 检测的DI状态
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 直线轨迹运动
	 */
	public int HRIF_MoveL_nj(int boxID, int rbtID,
						  double dX, double dY, double dZ, double dRx, double dRy, double dRz,
							 JointsDatas joints,
						  String sTcpName, String sUcsName,
						  double dvelocity, double dacc, double dradius, int nIsSeek, int nIOBit, int nIOState, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveL_nj_base(rbtID,
				dX, dY, dZ, dRx, dRy, dRz,
				joints,
				sTcpName, sUcsName,
				dvelocity, dacc, dradius, nIsSeek, nIOBit, nIOState, strCmdID);
	}

	/**
	 * @Title HRIF_MoveC
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dStartPose_X 圆弧起始点位置X
	 * @param dStartPose_Y 圆弧起始点位置Y
	 * @param dStartPose_Z 圆弧起始点位置Z
	 * @param dStartPose_Rx 圆弧起始点位置Rx
	 * @param dStartPose_Ry 圆弧起始点位置Ry
	 * @param dStartPose_Rz 圆弧起始点位置Rz
	 * @param dAuxPose_X 圆弧经过点位置X
	 * @param dAuxPose_Y 圆弧经过点位置Y
	 * @param dAuxPose_Z 圆弧经过点位置Z
	 * @param dAuxPose_Rx 圆弧经过点位置Rx
	 * @param dAuxPose_Ry 圆弧经过点位置Ry
	 * @param dAuxPose_Rz 圆弧经过点位置Rz
	 * @param dEndPose_X 圆弧结束点位置X
	 * @param dEndPose_Y 圆弧结束点位置Y
	 * @param dEndPose_Z 圆弧结束点位置Z
	 * @param dEndPose_Rx 圆弧结束点位置Rx
	 * @param dEndPose_Ry 圆弧结束点位置Ry
	 * @param dEndPose_Rz 圆弧结束点位置Rz
	 * @param nfixedPosure 是否固定姿态
	 * @param nMoveCType 圆弧类型
	 * @param dRadLen 弧长
	 * @param dVelocity 速度
	 * @param dAcceleration 加速度
	 * @param dRadius 过渡半径
	 * @param sTCPName 工具坐标名称
	 * @param sUCSName 用户坐标名称
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 圆弧轨迹运动
	 */
	public int HRIF_MoveC(int boxID, int rbtID,
			double dStartPose_X, double dStartPose_Y,double dStartPose_Z,double dStartPose_Rx,double dStartPose_Ry,double dStartPose_Rz,
			double dAuxPose_X, double dAuxPose_Y, double dAuxPose_Z, double dAuxPose_Rx, double dAuxPose_Ry, double dAuxPose_Rz,
			double dEndPose_X, double dEndPose_Y, double dEndPose_Z, double dEndPose_Rx, double dEndPose_Ry, double dEndPose_Rz,
			int nfixedPosure,
			int nMoveCType, double dRadLen, double dVelocity, double dAcceleration, double dRadius, String sTCPName, String sUCSName, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveC_base(rbtID,
				dStartPose_X, dStartPose_Y, dStartPose_Z, dStartPose_Rx, dStartPose_Ry, dStartPose_Rz,
				dAuxPose_X, dAuxPose_Y, dAuxPose_Z, dAuxPose_Rx, dAuxPose_Ry, dAuxPose_Rz,
				dEndPose_X, dEndPose_Y, dEndPose_Z, dEndPose_Rx, dEndPose_Ry, dEndPose_Rz,
				nfixedPosure, nMoveCType, dRadLen, dVelocity, dAcceleration, dRadius, sTCPName, sUCSName, strCmdID);
	}

	/**
	 * @Title HRIF_MoveC_nj
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dStartPose_X 圆弧起始点位置X
	 * @param dStartPose_Y 圆弧起始点位置Y
	 * @param dStartPose_Z 圆弧起始点位置Z
	 * @param dStartPose_Rx 圆弧起始点位置Rx
	 * @param dStartPose_Ry 圆弧起始点位置Ry
	 * @param dStartPose_Rz 圆弧起始点位置Rz
	 * @param dAuxPose_X 圆弧经过点位置X
	 * @param dAuxPose_Y 圆弧经过点位置Y
	 * @param dAuxPose_Z 圆弧经过点位置Z
	 * @param dAuxPose_Rx 圆弧经过点位置Rx
	 * @param dAuxPose_Ry 圆弧经过点位置Ry
	 * @param dAuxPose_Rz 圆弧经过点位置Rz
	 * @param dEndPose_X 圆弧结束点位置X
	 * @param dEndPose_Y 圆弧结束点位置Y
	 * @param dEndPose_Z 圆弧结束点位置Z
	 * @param dEndPose_Rx 圆弧结束点位置Rx
	 * @param dEndPose_Ry 圆弧结束点位置Ry
	 * @param dEndPose_Rz 圆弧结束点位置Rz
	 * @param nfixedPosure 是否固定姿态
	 * @param nMoveCType 圆弧类型
	 * @param dRadLen 弧长
	 * @param dVelocity 速度
	 * @param dAcceleration 加速度
	 * @param dRadius 过渡半径
	 * @param sTCPName 工具坐标名称
	 * @param sUCSName 用户坐标名称
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 圆弧轨迹运动
	 */
	public int HRIF_MoveC_nj(int boxID, int rbtID,
						  double dStartPose_X, double dStartPose_Y,double dStartPose_Z,double dStartPose_Rx,double dStartPose_Ry,double dStartPose_Rz,
						  double dAuxPose_X, double dAuxPose_Y, double dAuxPose_Z, double dAuxPose_Rx, double dAuxPose_Ry, double dAuxPose_Rz,
						  double dEndPose_X, double dEndPose_Y, double dEndPose_Z, double dEndPose_Rx, double dEndPose_Ry, double dEndPose_Rz,
						  int nfixedPosure,
						  int nMoveCType, double dRadLen, double dVelocity, double dAcceleration, double dRadius, String sTCPName, String sUCSName, String strCmdID,
							 JointsDatas EndPointRefACS, int nIsUseCurRefACS )
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveC_nj_base(rbtID,
				dStartPose_X, dStartPose_Y, dStartPose_Z, dStartPose_Rx, dStartPose_Ry, dStartPose_Rz,
				dAuxPose_X, dAuxPose_Y, dAuxPose_Z, dAuxPose_Rx, dAuxPose_Ry, dAuxPose_Rz,
				dEndPose_X, dEndPose_Y, dEndPose_Z, dEndPose_Rx, dEndPose_Ry, dEndPose_Rz,
				nfixedPosure, nMoveCType, dRadLen, dVelocity, dAcceleration, dRadius, sTCPName, sUCSName, strCmdID,EndPointRefACS,nIsUseCurRefACS);
	}

	/**
	 * @Title HRIF_MoveZ
	 * @date 2022年6月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
     * @param dStartPose_X Z型起始点位置X
	 * @param dStartPose_Y Z型起始点位置Y
	 * @param dStartPose_Z Z型起始点位置Z
	 * @param dStartPose_Rx Z型起始点位置Rx
	 * @param dStartPose_Ry Z型起始点位置Ry
	 * @param dStartPose_Rz Z型起始点位置Rz
	 * @param dEndPose_X Z型结束点位置X
	 * @param dEndPose_Y Z型结束点位置Y
	 * @param dEndPose_Z Z型结束点位置Z
	 * @param dEndPose_Rx Z型结束点位置Rx
	 * @param dEndPose_Ry Z型结束点位置Ry
	 * @param dEndPose_Rz Z型结束点位置Rz
	 * @param dPlanePos_X 轨迹确定平面点位置X
	 * @param dPlanePos_Y 轨迹确定平面点位置Y
	 * @param dPlanePos_Z 轨迹确定平面点位置Z
	 * @param dPlanePos_Rx 轨迹确定平面点位置Rx
	 * @param dPlanePos_Ry 轨迹确定平面点位置Ry
	 * @param dPlanePos_Rz 轨迹确定平面点位置Rz
	 * @param dVelocity 速度
	 * @param dacc 加速度
	 * @param dwidth 宽度
	 * @param ddensity 密度
	 * @param nEnableDensity 是否使用密度
	 * @param nEnablePlane 是否使用平面点
	 * @param nEnableWaitTime 是否开启转折点等待时间
	 * @param nPosiTime 正向转折点等待时间ms
	 * @param nNegaTime 负向转折点等待时间ms
	 * @param dradius 过渡半径
	 * @param sTcpName 工具坐标名称
	 * @param sUcsName 用户坐标名称
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description Z型轨迹运动
	 */
	public int HRIF_MoveZ(int boxID, int rbtID,
			double dStartPose_X, double dStartPose_Y, double dStartPose_Z, double dStartPose_Rx, double dStartPose_Ry, double dStartPose_Rz,
			double dEndPose_X, double dEndPose_Y, double dEndPose_Z, double dEndPose_Rx, double dEndPose_Ry, double dEndPose_Rz,
			double dPlanePos_X, double dPlanePos_Y, double dPlanePos_Z, double dPlanePos_Rx, double dPlanePos_Ry, double dPlanePos_Rz,
			double dVelocity,
			double dacc, double dwidth, double ddensity, int nEnableDensity, int nEnablePlane, int nEnableWaitTime,
			int nPosiTime, int nNegaTime, double dradius, String sTcpName, String sUcsName, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveZ_base(rbtID,
				dStartPose_X, dStartPose_Y, dStartPose_Z, dStartPose_Rx, dStartPose_Ry, dStartPose_Rz,
				dEndPose_X, dEndPose_Y, dEndPose_Z, dEndPose_Rx, dEndPose_Ry, dEndPose_Rz,
				dPlanePos_X, dPlanePos_Y, dPlanePos_Z, dPlanePos_Rx, dPlanePos_Ry, dPlanePos_Rz,
				dVelocity, dacc, dwidth, ddensity, nEnableDensity, nEnablePlane, nEnableWaitTime,
				nPosiTime, nNegaTime, dradius, sTcpName, sUcsName, strCmdID);
	}

	/**
	 * @Title HRIF_MoveE
	 * @date 2022年6月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dP1_X 示教位置1-X
	 * @param dP1_Y 示教位置1-Y
	 * @param dP1_Z 示教位置1-Z
	 * @param dP1_Rx 示教位置1-Rx
	 * @param dP1_Ry 示教位置1-Ry
	 * @param dP1_Rz 示教位置1-Rz
 	 * @param dP2_X 示教位置2-X
	 * @param dP2_Y 示教位置2-Y
	 * @param dP2_Z 示教位置2-Z
	 * @param dP2_Rx 示教位置2-Rx
	 * @param dP2_Ry 示教位置2-Ry
	 * @param dP2_Rz 示教位置2-Rz
 	 * @param dP3_X 示教位置3-X
	 * @param dP3_Y 示教位置3-Y
	 * @param dP3_Z 示教位置3-Z
	 * @param dP3_Rx 示教位置3-Rx
	 * @param dP3_Ry 示教位置3-Ry
	 * @param dP3_Rz 示教位置3-Rz
 	 * @param dP4_X 示教位置4-X
	 * @param dP4_Y 示教位置4-Y
	 * @param dP4_Z 示教位置4-Z
	 * @param dP4_Rx 示教位置4-Rx
	 * @param dP4_Ry 示教位置4-Ry
	 * @param dP4_Rz 示教位置4-Rz
 	 * @param dP5_X 示教位置5-X
	 * @param dP5_Y 示教位置5-Y
	 * @param dP5_Z 示教位置5-Z
	 * @param dP5_Rx 示教位置5-Rx
	 * @param dP5_Ry 示教位置5-Ry
	 * @param dP5_Rz 示教位置5-Rz
	 * @param nMoveType 运动类型
	 * @param nOrientMode 运动模式
	 * @param dArcLength 弧长
	 * @param dvelocity 速度
	 * @param dacc 加速度
	 * @param dradius 过渡半径
	 * @param sTcpName 工具坐标名称
	 * @param sUcsName 用户坐标名称
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 椭圆型轨迹运动
	 */
	public int HRIF_MoveE(int boxID, int rbtID,
			double dP1_X, double dP1_Y, double dP1_Z, double dP1_Rx, double dP1_Ry, double dP1_Rz,
			double dP2_X, double dP2_Y, double dP2_Z, double dP2_Rx, double dP2_Ry, double dP2_Rz,
			double dP3_X, double dP3_Y, double dP3_Z, double dP3_Rx, double dP3_Ry, double dP3_Rz,
			double dP4_X, double dP4_Y, double dP4_Z, double dP4_Rx, double dP4_Ry, double dP4_Rz,
			double dP5_X, double dP5_Y, double dP5_Z, double dP5_Rx, double dP5_Ry, double dP5_Rz,
			int nMoveType, int nOrientMode, double dArcLength,
			double dvelocity, double dacc, double dradius, String sTcpName, String sUcsName, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveE_base(rbtID, dP1_X, dP1_Y, dP1_Z, dP1_Rx, dP1_Ry, dP1_Rz,
				 dP2_X, dP2_Y, dP2_Z, dP2_Rx, dP2_Ry, dP2_Rz,
				 dP3_X, dP3_Y, dP3_Z, dP3_Rx, dP3_Ry, dP3_Rz,
				 dP4_X, dP4_Y, dP4_Z, dP4_Rx, dP4_Ry, dP4_Rz,
				 dP5_X, dP5_Y, dP5_Z, dP5_Rx, dP5_Ry, dP5_Rz,
				 nMoveType,nOrientMode,  dArcLength,
				 dvelocity, dacc, dradius, sTcpName, sUcsName, strCmdID);

	}

	/**
	 * @Title HRIF_MoveS
	 * @date 2022年6月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dSpiralIncrement 增量半径
	 * @param dSpiralDiameter 结束半径
	 * @param dvelocity 速度
	 * @param dacc 加速度
	 * @param dradius 过渡半径
	 * @param sTcpName 工具坐标名称
	 * @param sUcsName 用户坐标名称
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 螺旋轨迹运动
	 */
	public int HRIF_MoveS(int boxID, int rbtID, double dSpiralIncrement, double dSpiralDiameter, double dvelocity,
			double dacc, double dradius, String sTcpName, String sUcsName, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveS_base(rbtID, dSpiralIncrement, dSpiralDiameter, dvelocity,
				 dacc, dradius, sTcpName, sUcsName, strCmdID);
	}

	/**
	 * @Title HRIF_MoveAlignToZ
	 * @date 2024年9月29日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTcpName 工具坐标名称
	 * @param sUcsName 用户坐标名称
	 * @param bIsReached z轴是否对齐标志
	 * @param dbJ1 J1
	 * @param dbJ2 J2
	 * @param dbJ3 J3
	 * @param dbJ4 J4
	 * @param dbJ5 J5
	 * @param dbJ6 J6
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description Z轴对齐
	 */
	public int HRIF_MoveAlignToZ(int boxID, int rbtID, String sTcpName, String sUcsName,BooleanData bIsReached,DoubleData dbJ1,DoubleData dbJ2,DoubleData dbJ3,DoubleData dbJ4,DoubleData dbJ5,DoubleData dbJ6)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveAlignToZ_base(rbtID,sTcpName,sUcsName,bIsReached,dbJ1,dbJ2,dbJ3,dbJ4,dbJ5,dbJ6);
	}

	/**
	 * @Title HRIF_MoveLinearWeave
	 * @date 2024年9月9日
	 * @param rbtID             机器人ID
	 * @param StartPoint 开始点位置
	 * @param EndPoint 结束点位置
	 * @param dvelocity 速度
	 * @param dAcc 加速度
	 * @param dRadius 过渡半径
	 * @param dAmplitude 宽度
	 * @param dInterValDistance 间距
	 * @param nWeaveFrameType 选择方式
	 * @param dElevation 仰角
	 * @param dAzimuth 方向角
	 * @param dCentreRise 中心隆起量
	 * @param nEnableWaitTime 是否等待
	 * @param nPosiTime 正等待时间
	 * @param nNegaTime 负等待时间
	 * @param sTcpName 工具坐标名称
	 * @param sUcsName 用户坐标名称
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 直线摆焊运动
	 */
	public int HRIF_MoveLinearWeave(int boxID, int rbtID, String StartPoint, String EndPoint, double dvelocity,
						  double dAcc, double dRadius, double dAmplitude, double dInterValDistance, int nWeaveFrameType,
									double dElevation,double dAzimuth,double dCentreRise,int nEnableWaitTime,double nPosiTime,
									double nNegaTime,String sTcpName,String sUcsName, String strCmdID,StringData vecLastPoint	)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveLinearWeave_base(rbtID, StartPoint, EndPoint, dvelocity,
				dAcc, dRadius,dAmplitude,dInterValDistance,nWeaveFrameType, dElevation,dAzimuth,dCentreRise,
				nEnableWaitTime,nPosiTime,nNegaTime,sTcpName, sUcsName, strCmdID,vecLastPoint);
	}

	/**
	 * @Title HRIF_MoveLinearWeave
	 * @date 2024年9月9日
	 * @param rbtID             机器人ID
	 * @param StartPoint 开始点位置
	 * @param AuxPoint 经过点位置
	 * @param EndPoint 结束点位置
	 * @param dvelocity 速度
	 * @param dAcc 加速度
	 * @param dRadius 过渡半径
	 * @param nOrientMode 是否使用固定姿态
	 * @param nMoveWhole 轨迹
	 * @param nMoveWholeLen 圈数
	 * @param dAmplitude 宽度
	 * @param dInterValDistance 间距
	 * @param nWeaveFrameType 选择方式
	 * @param dElevation 仰角
	 * @param dAzimuth 方向角
	 * @param dCentreRise 中心隆起量
	 * @param nEnableWaitTime 是否等待
	 * @param nPosiTime 正等待时间
	 * @param nNegaTime 负等待时间
	 * @param sTcpName 工具坐标名称
	 * @param sUcsName 用户坐标名称
	 * @param strCmdID 命令ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 直线摆焊运动
	 */
	public int HRIF_MoveCircularWeave(int boxID, int rbtID, String StartPoint,String AuxPoint, String EndPoint, double dvelocity,
									double dAcc, double dRadius,int nOrientMode,int nMoveWhole,int nMoveWholeLen, double dAmplitude, double dInterValDistance, int nWeaveFrameType,
									double dElevation,double dAzimuth,double dCentreRise,int nEnableWaitTime,double nPosiTime,
									double nNegaTime,String sTcpName,String sUcsName, String strCmdID,StringData vecLastPoint	)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveCircularWeave_base(rbtID, StartPoint,AuxPoint, EndPoint, dvelocity,
				dAcc, dRadius,nOrientMode, nMoveWhole, nMoveWholeLen,dAmplitude,dInterValDistance,nWeaveFrameType, dElevation,dAzimuth,dCentreRise,
				nEnableWaitTime,nPosiTime,nNegaTime,sTcpName, sUcsName, strCmdID,vecLastPoint);
	}

	/**
	 * @Title HRIF_MoveRelJ
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nAxisID 运动的目标轴ID 对应J1-J6
	 * @param nDirection 运动方向 0:负方向/1:正方向
	 * @param dDistance 相对运动距离
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 关节相对运动
	 */
	public int HRIF_MoveRelJ(int boxID, int rbtID, int nAxisID, int nDirection, double dDistance)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveRelJ_base(rbtID, nAxisID, nDirection, dDistance);
	}

	/**
	 * @Title HRIF_MoveRelL
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nAxisID 运动的目标轴 ID，对应空间坐标 X-Rz
	 * @param nDirection 运动方向 0:负方向 1:正方向
	 * @param dDistance 相对运动距离
	 * @param nToolMotion 运动坐标类型 0：按当前选择的用户坐标运动/1：按 Tool 坐标运动
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 空间相对运动
	 */
	public int HRIF_MoveRelL(int boxID, int rbtID, int nAxisID, int nDirection, double dDistance, int nToolMotion)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MoveRelL_base(rbtID, nAxisID, nDirection, dDistance, nToolMotion);
	}

	/**
	 * @Title HRIF_WayPointRel
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nType 运动类型	0：关节相对运动/1：直线相对运动
	 * @param nPointList 是否使用点位列表点位 0：不使用点位列表中点位/1：使用点位列表中点位
	 * @param dPos_X X坐标，单位[mm]
	 * @param dPos_Y Y坐标，单位[mm]
	 * @param dPos_Z Z坐标，单位[mm]
	 * @param dPos_Rx Rx坐标，单位[°]
	 * @param dPos_Ry Ry坐标，单位[°]
	 * @param dPos_Rz Rz坐标，单位[°]
	 * @param dPos_J1 关节1坐标，单位[°]
	 * @param dPos_J2 关节2坐标，单位[°]
	 * @param dPos_J3 关节3坐标，单位[°]
	 * @param dPos_J4 关节4坐标，单位[°]
	 * @param dPos_J5 关节5坐标，单位[°]
	 * @param dPos_J6 关节6坐标，单位[°]
	 * @param nrelMoveType 相对运动类型 0：绝对值/1：叠加值
	 * @param nAxisMask_1 是否运动 0:不运动/1:运动;nType=0时，表示笛卡尔方向索引;nType=1时，表示关节索引;
	 * @param nAxisMask_2 是否运动 0:不运动/1:运动;nType=0时，表示笛卡尔方向索引;nType=1时，表示关节索引;
	 * @param nAxisMask_3 是否运动 0:不运动/1:运动;nType=0时，表示笛卡尔方向索引;nType=1时，表示关节索引;
	 * @param nAxisMask_4 是否运动 0:不运动/1:运动;nType=0时，表示笛卡尔方向索引;nType=1时，表示关节索引;
	 * @param nAxisMask_5 是否运动 0:不运动/1:运动;nType=0时，表示笛卡尔方向索引;nType=1时，表示关节索引;
	 * @param nAxisMask_6 是否运动 0:不运动/1:运动;nType=0时，表示笛卡尔方向索引;nType=1时，表示关节索引;
	 * @param dTarget1 运动距离 nType=0 并 nAxisMask=1：该方向运动绝对距离或叠加距离;nType=1 并 nAxisMask=1：该轴运动绝对距离或叠加距离;nAxisMask=0：无效
	 * @param dTarget2 运动距离 nType=0 并 nAxisMask=1：该方向运动绝对距离或叠加距离;nType=1 并 nAxisMask=1：该轴运动绝对距离或叠加距离;nAxisMask=0：无效
	 * @param dTarget3 运动距离 nType=0 并 nAxisMask=1：该方向运动绝对距离或叠加距离;nType=1 并 nAxisMask=1：该轴运动绝对距离或叠加距离;nAxisMask=0：无效
	 * @param dTarget4 运动距离 nType=0 并 nAxisMask=1：该方向运动绝对距离或叠加距离;nType=1 并 nAxisMask=1：该轴运动绝对距离或叠加距离;nAxisMask=0：无效
	 * @param dTarget5 运动距离 nType=0 并 nAxisMask=1：该方向运动绝对距离或叠加距离;nType=1 并 nAxisMask=1：该轴运动绝对距离或叠加距离;nAxisMask=0：无效
	 * @param dTarget6 运动距离 nType=0 并 nAxisMask=1：该方向运动绝对距离或叠加距离;nType=1 并 nAxisMask=1：该轴运动绝对距离或叠加距离;nAxisMask=0：无效
	 * @param sTcpName 工具坐标名称 目标空间坐标所处的工具坐标系名称，与示教器页面的名称对应，当 nIsUseJoint=1 时无效，可使用默认名称 “TCP”
	 * @param sUCSName 用户坐标名称 目标空间坐标所处的用户坐标系名称，与示教器页面的名称对应，当 nIsUseJoint=1 时无效，可使用默认名称 “Base”
	 * @param dVelocity 速度 运动最大速度，关节运动时单位[°/s]，空间运动时 X，Y，Z 单位[mm/s]，Rx，Ry，Rz 单位[°/s]
	 * @param dAcc 加速度 运动最大加速度，关节运动时单位[°/s2]，空间运动时X，Y，Z 单位[mm/s2]，Rx，Ry，Rz 单位[°/s2]
	 * @param dRadius 过渡半径，单位[mm]
	 * @param nIsUseJoint 默认为0，其他值无效，使用关节角度
	 * @param nIsSeek 如果 nIsSeek 为1，则开启检测 DI 停止，路点运动过程中如果电箱的 nIOBit 位索引的 DI 的状态=nIOState 时，机器人停止运动，否则运动到目标点完成运动
	 * @param nIOBit 电箱对应 DI 索引，nIsSeek=0 时无效
	 * @param nIOState 检测的 DI 状态，，nIsSeek=0 时无效
	 * @param strCmdID 当前路点 ID，可以自定义，也可以按顺序设置为“1”,“2”,“3”.
	 * @return int, 0：调用成功;>0返回错误码
	 */
	public int HRIF_WayPointRel(int boxID,int rbtID, int nType, int nPointList, double dPos_X, double dPos_Y, double dPos_Z, double dPos_Rx, double dPos_Ry, double dPos_Rz,
			double dPos_J1,double dPos_J2, double dPos_J3, double dPos_J4,double dPos_J5, double dPos_J6, int nrelMoveType,
			int nAxisMask_1,int nAxisMask_2,int nAxisMask_3,int nAxisMask_4,int nAxisMask_5,int nAxisMask_6,
			double dTarget1,double dTarget2,double dTarget3,double dTarget4,double dTarget5,double dTarget6,
			String sTcpName, String sUCSName, double dVelocity, double dAcc, double dRadius, int nIsUseJoint,
			int nIsSeek, int nIOBit, int nIOState, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_WayPointRel_base(rbtID, nType, nPointList, dPos_X, dPos_Y, dPos_Z, dPos_Rx, dPos_Ry, dPos_Rz,
				dPos_J1, dPos_J2, dPos_J3, dPos_J4, dPos_J5, dPos_J6, nrelMoveType,
				 nAxisMask_1, nAxisMask_2, nAxisMask_3, nAxisMask_4, nAxisMask_5, nAxisMask_6,
				 dTarget1, dTarget2, dTarget3, dTarget4, dTarget5, dTarget6,
				 sTcpName, sUCSName, dVelocity, dAcc, dRadius, nIsUseJoint,
				 nIsSeek, nIOBit, nIOState, strCmdID);
	}

	/**
	 * @Title HRIF_WayPointRel_nj
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nType 运动类型	0：关节相对运动/1：直线相对运动
	 * @param nPointList 是否使用点位列表点位 0：不使用点位列表中点位/1：使用点位列表中点位
	 * @param dPos_X X坐标，单位[mm]
	 * @param dPos_Y Y坐标，单位[mm]
	 * @param dPos_Z Z坐标，单位[mm]
	 * @param dPos_Rx Rx坐标，单位[°]
	 * @param dPos_Ry Ry坐标，单位[°]
	 * @param dPos_Rz Rz坐标，单位[°]
	 * @param nrelMoveType 相对运动类型 0：绝对值/1：叠加值
	 * @param dTarget1 运动距离 nType=0 并 nAxisMask=1：该方向运动绝对距离或叠加距离;nType=1 并 nAxisMask=1：该轴运动绝对距离或叠加距离;nAxisMask=0：无效
	 * @param dTarget2 运动距离 nType=0 并 nAxisMask=1：该方向运动绝对距离或叠加距离;nType=1 并 nAxisMask=1：该轴运动绝对距离或叠加距离;nAxisMask=0：无效
	 * @param dTarget3 运动距离 nType=0 并 nAxisMask=1：该方向运动绝对距离或叠加距离;nType=1 并 nAxisMask=1：该轴运动绝对距离或叠加距离;nAxisMask=0：无效
	 * @param dTarget4 运动距离 nType=0 并 nAxisMask=1：该方向运动绝对距离或叠加距离;nType=1 并 nAxisMask=1：该轴运动绝对距离或叠加距离;nAxisMask=0：无效
	 * @param dTarget5 运动距离 nType=0 并 nAxisMask=1：该方向运动绝对距离或叠加距离;nType=1 并 nAxisMask=1：该轴运动绝对距离或叠加距离;nAxisMask=0：无效
	 * @param dTarget6 运动距离 nType=0 并 nAxisMask=1：该方向运动绝对距离或叠加距离;nType=1 并 nAxisMask=1：该轴运动绝对距离或叠加距离;nAxisMask=0：无效
	 * @param sTcpName 工具坐标名称 目标空间坐标所处的工具坐标系名称，与示教器页面的名称对应，当 nIsUseJoint=1 时无效，可使用默认名称 “TCP”
	 * @param sUCSName 用户坐标名称 目标空间坐标所处的用户坐标系名称，与示教器页面的名称对应，当 nIsUseJoint=1 时无效，可使用默认名称 “Base”
	 * @param dVelocity 速度 运动最大速度，关节运动时单位[°/s]，空间运动时 X，Y，Z 单位[mm/s]，Rx，Ry，Rz 单位[°/s]
	 * @param dAcc 加速度 运动最大加速度，关节运动时单位[°/s2]，空间运动时X，Y，Z 单位[mm/s2]，Rx，Ry，Rz 单位[°/s2]
	 * @param dRadius 过渡半径，单位[mm]
	 * @param nIsUseJoint 默认为0，其他值无效，使用关节角度
	 * @param nIsSeek 如果 nIsSeek 为1，则开启检测 DI 停止，路点运动过程中如果电箱的 nIOBit 位索引的 DI 的状态=nIOState 时，机器人停止运动，否则运动到目标点完成运动
	 * @param nIOBit 电箱对应 DI 索引，nIsSeek=0 时无效
	 * @param nIOState 检测的 DI 状态，，nIsSeek=0 时无效
	 * @param strCmdID 当前路点 ID，可以自定义，也可以按顺序设置为“1”,“2”,“3”.
	 * @return int, 0：调用成功;>0返回错误码
	 */
	public int HRIF_WayPointRel_nj(int boxID,int rbtID, int nType, int nPointList, double dPos_X, double dPos_Y, double dPos_Z, double dPos_Rx, double dPos_Ry, double dPos_Rz,
								   JointsDatas Pos, int nrelMoveType, JointsDatas Axis,
								double dTarget1,double dTarget2,double dTarget3,double dTarget4,double dTarget5,double dTarget6,
								String sTcpName, String sUCSName, double dVelocity, double dAcc, double dRadius, int nIsUseJoint,
								int nIsSeek, int nIOBit, int nIOState, String strCmdID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_WayPointRel_nj_base(rbtID, nType, nPointList, dPos_X, dPos_Y, dPos_Z, dPos_Rx, dPos_Ry, dPos_Rz,
				Pos, nrelMoveType, Axis,
				dTarget1, dTarget2, dTarget3, dTarget4, dTarget5, dTarget6,
				sTcpName, sUCSName, dVelocity, dAcc, dRadius, nIsUseJoint,
				nIsSeek, nIOBit, nIOState, strCmdID);
	}

	/**-----------------------  Part11 连续轨迹运动类控制指令-------------------------*/

	/**
	 * @Title HRIF_InitPath
	 * @date 2024年8月30日
	 * @param rbtID 机器人ID
	 * @param boxID 电箱ID
	 * @param nRawDataType 轨迹原始点位类型
	 * @param trajectName 轨迹名称
	 * @param dSpeedRatio 轨迹运动速度比
	 * @param dRadius 过渡半径
	 * @param dVelocity  轨迹运动速度
	 * @param dAcc 轨迹运行加速度
	 * @param dJerk 轨迹运动加加速度
	 * @param sUcsName 用户坐标名称
	 * @param sTcpName 工具坐标名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 初始化新建轨迹
	 */
	public int HRIF_InitPath(int boxID, int rbtID, int nRawDataType,String trajectName, double dSpeedRatio, double dRadius,double dVelocity,double dAcc,double dJerk,String sUcsName,String sTcpName)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_InitPath_base(rbtID,nRawDataType,trajectName,dSpeedRatio,dRadius,dVelocity,dAcc,dJerk,sUcsName,sTcpName);
	}

	/**
	 * @Title HRIF_PushPathPoints
	 * @date 2024年8月30日
	 * @param rbtID 机器人ID
	 * @param boxID 电箱ID
	 * @param sPoints 点位数据
	 * @param trajectName 轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 向轨迹中批量推送原始点位
	 */
	public int HRIF_PushPathPoints(int boxID, int rbtID, String trajectName, String sPoints)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PushPathPoints_base(rbtID,trajectName,sPoints);
	}

	/**
	 * @Title HRIF_EndPushPathPoints
	 * @date 2024年8月30日
	 * @param rbtID 机器人ID
	 * @param boxID 电箱ID
	 * @param sPathName 轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 结束向轨迹中推送点位
	 */
	public int HRIF_EndPushPathPoints(int boxID, int rbtID, String sPathName)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_EndPushPathPoints_base(rbtID,sPathName);
	}

	/**
	 * @Title HRIF_MovePathJOL
	 * @date 2024年9月11日
	 * @param rbtID 机器人ID
	 * @param boxID 电箱ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 启动在线实施规划的MovePathJ
	 */
	public int HRIF_MovePathJOL(int boxID, int rbtID,double dVel,double dAcc,double dTol,String RawACSpoints,String nIsSetIO,
								String nEndDOMask,String nEndDOVal, String nBoxDOMask,String nBoxDOVal,
								String nBoxCOMask,String nBoxCOVal,String nBoxAOCH0_Mask,String nBoxAOCH0_Mode,String nBoxAOCH1_Mask,
								String nBoxAOCH1_Mode,String dbBoxAOCH0_Val,String dbBoxAOCH1_Val)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MovePathJOL_base(rbtID,dVel, dAcc, dTol, RawACSpoints, nIsSetIO, nEndDOMask, nEndDOVal, nBoxDOMask,
				nBoxDOVal, nBoxCOMask, nBoxCOVal, nBoxAOCH0_Mask, nBoxAOCH0_Mode, nBoxAOCH1_Mask, nBoxAOCH1_Mode, dbBoxAOCH0_Val, dbBoxAOCH1_Val);
	}

	/**
	 * @Title HRIF_DelPath
	 * @date 2024年8月31日
	 * @param rbtID 机器人ID
	 * @param boxID 电箱ID
	 * @param sPathName 轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 删除指定轨迹
	 */
	public int HRIF_DelPath(int boxID, int rbtID, String sPathName)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_DelPath_base(rbtID,sPathName);
	}

	/**
	 * @Title HRIF_ReadPathList
	 * @date 2024年9月11日
	 * @param rbtID 机器人ID
	 * @param boxID 电箱ID
	 * @param result 轨迹信息
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 读取轨迹列表
	 */
	public int HRIF_ReadPathList(int boxID, int rbtID , StringData result)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadPathList_base(rbtID,result);
	}

	/**
	 * @Title HRIF_ReadPathInfo
	 * @date 2024年8月31日
	 * @param rbtID 机器人ID
	 * @param boxID 电箱ID
	 * @param sPathName 轨迹名称
	 * @param result 轨迹信息
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 读取轨迹信息
	 */
	public int HRIF_ReadPathInfo(int boxID, int rbtID, String sPathName, StringData result)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadPathInfo_base(rbtID,sPathName,result);
	}

	/**
	 * @Title HRIF_UpdatePathName
	 * @date 2024年8月31日
	 * @param rbtID 机器人ID
	 * @param boxID 电箱ID
	 * @param sPathName 原轨迹名称
	 * @param sPathNewName 新轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 更新指定轨迹的名称
	 */
	public int HRIF_UpdatePathName(int boxID, int rbtID, String sPathName, String sPathNewName )
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_UpdatePathName_base(rbtID,sPathName,sPathNewName );
	}

	/**
	 * @Title HRIF_ReadPathState
	 * @date 2024年8月31日
	 * @param rbtID 机器人ID
	 * @param boxID 电箱ID
	 * @param sPathName 原轨迹名称
	 * @param nStateJ 轨迹的MovePathJ状态
	 * @param nErrorCodeJ MovePathJ状态对应的错误码
	 * @param nStateL 轨迹的MovePathL状态
	 * @param nErrorCodeL MovePathL状态对应的错误码
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 读取当前轨迹状态
	 */
	public int HRIF_ReadPathState(int boxID, int rbtID, String sPathName, IntData nStateJ, IntData nErrorCodeJ,  IntData nStateL,
								  IntData nErrorCodeL)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadPathState_base(rbtID,sPathName, nStateJ, nErrorCodeJ, nStateL,nErrorCodeL );
	}

	/**
	 * @Title HRIF_StartPushMovePathJ
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @param dSpeedRatio 轨迹运动速度比
	 * @param dRadius 过渡半径
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 初始化关节连续轨迹运动
	 */
	public int HRIF_StartPushMovePathJ(int boxID, int rbtID, String sTrackName, double dSpeedRatio, double dRadius)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_StartPushMovePathJ_base(rbtID, sTrackName, dSpeedRatio, dRadius);
	}

	/**
	 * @Title HRIF_PushMovePathJ
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @param dJ1 关节1位置,单位[°]
	 * @param dJ2 关节2位置,单位[°]
	 * @param dJ3 关节3位置,单位[°]
	 * @param dJ4 关节4位置,单位[°]
	 * @param dJ5 关节5位置,单位[°]
	 * @param dJ6 关节6位置,单位[°]
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 下发运动轨迹点位置
	 * @Attention 调用HRIF_StartPushMovePath后，可多次调用此函数，一般情况下点位数量需要>4。
	 */
	public int HRIF_PushMovePathJ(int boxID, int rbtID, String sTrackName, double dJ1, double dJ2, double dJ3, double dJ4, double dJ5,double dJ6)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PushMovePathJ_base(rbtID, sTrackName, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_EndPushMovePathJ
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 轨迹下发完成并开始计算轨迹
	 */
	public int HRIF_EndPushMovePathJ(int boxID, int rbtID, String sTrackName)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_EndPushMovePathJ_base(rbtID, sTrackName);
	}

	/**
	 * @Title HRIF_MovePathJ
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 运动指定的轨迹
	 */
	public int HRIF_MovePathJ(int boxID, int rbtID, String sTrackName)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MovePathJ_base(rbtID, sTrackName);
	}

	/**
	 * @Title HRIF_ReadMovePathState
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @param nState 轨迹状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前的轨迹状态
	 */
	public int HRIF_ReadMovePathJState(int boxID, int rbtID, String sTrackName, IntData nState)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadMovePathJState_base(rbtID, sTrackName, nState);
	}

	/**
	 * @Title HRIF_UpdateMovePathJName
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹原名称
	 * @param sTrackNewName 更新的轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 更新指定轨迹的名称
	 */
	public int HRIF_UpdateMovePathJName(int boxID, int rbtID, String sTrackName, String sTrackNewName)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_UpdateMovePathJName_base(rbtID, sTrackName, sTrackNewName);
	}

	/**
	 * @Title HRIF_DelMovePath
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹原名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 删除指定轨迹
	 */
	public int HRIF_DelMovePathJ(int boxID, int rbtID, String sTrackName)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_DelMovePathJ_base(rbtID, sTrackName);
	}

	/**
	 * @Title HRIF_GetMovePathJOLIndex
	 * @date 2024年9月11日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param getIndexForPoint 点位索引号
	 * @param getTotalPoints 总点数
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 获取MovePathJOL运动当前的点位索引号及轨迹运动所有点总数。
	 */
	public int HRIF_GetMovePathJOLIndex(int boxID, int rbtID, IntData getIndexForPoint,IntData getTotalPoints)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_GetMovePathJOLIndex_base(rbtID,getIndexForPoint, getTotalPoints);
	}

	/**
	 * @Title HRIF_ReadTrackProcess
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dProcess 轨迹运行进度
	 * @param nIndex 点位索引
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前的轨迹运动进度
	 */
	public int HRIF_ReadTrackProcess(int boxID, int rbtID, DoubleData dProcess, IntData nIndex)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadTrackProcess_base(rbtID, dProcess, nIndex);
	}

	/**
	 * @Title HRIF_InitMovePathL
	 * @date 2022年6月10日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @param dvelocity 轨迹运动速度
	 * @param dacceleration 轨迹运动加速度
	 * @param djerk 轨迹运动加加速度
	 * @param sUcsName 工具坐标名称
	 * @param sTcpName 用户坐标名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 初始化空间轨迹运动
	 */
	public int HRIF_InitMovePathL(int boxID, int rbtID, String sTrackName, double dvelocity, double dacceleration, double djerk,
			String sUcsName, String sTcpName) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_InitMovePathL_base(rbtID, sTrackName, dvelocity, dacceleration, djerk, sUcsName, sTcpName);
	}

	/**
	 * @Title HRIF_PushMovePathL
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @param dx X坐标,单位[mm]
	 * @param dy Y坐标,单位[mm]
	 * @param dz Z坐标,单位[mm]
	 * @param drx RX坐标,单位[mm]
	 * @param dry RY坐标,单位[mm]
	 * @param drz RZ坐标,单位[mm]
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 下发运动轨迹点位，
	 */
	public int HRIF_PushMovePathL(int boxID, int rbtID, String sTrackName, double dx, double dy, double dz, double drx, double dry,
			double drz) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PushMovePathL_base(rbtID, sTrackName, dx, dy, dz, drx, dry, drz);
	}

	/**
	 * @Title HRIF_PushMovePaths
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @param nMoveType 点位类型
	 * @param nPointsSize 点位数量
	 * @param sPoints 点位数据
	 * @return int, 0:调用成功;>0返回错误码
	 * @Description 批量下发轨迹点位，调用一次可下发多个点位数据
	 */
	public int HRIF_PushMovePaths(int boxID, int rbtID, String sTrackName, int nMoveType, int nPointsSize, String sPoints)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PushMovePaths_base(rbtID, sTrackName, nMoveType, nPointsSize, sPoints);
	}

	/**
	 * @Title HRIF_MovePathL
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @return int, 0:调用成功;>0返回错误码
	 * @Description 执行空间坐标轨迹运动
	 * @Attention 调用HRIF_MovePathL后会计算完轨迹后直接开始运动，计算时间2-4s左右，根据实际轨迹大小确定。
	 */
	public int HRIF_MovePathL(int boxID, int rbtID, String sTrackName)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_MovePathL_base(rbtID, sTrackName);
	}

	/**
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dOverride 速度比
	 * @return int, 0:调用成功;>0返回错误码
	 * @Description 设置 MovePath 运动速度比
	 */
	public int HRIF_SetMovePathOverride (int boxID, int rbtID, double dOverride)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetMovePathOverride_base(rbtID, dOverride);
	}

	/**-----------------------Part12 Servo运动类控制指令-------------------------*/

	/**
	 * @Title HRIF_StartServo
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dservoTime 更新周期
	 * @param dlookaheadTime 前瞻时间
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 启动机器人在线控制（ servoJ 或 servoP）时，设定位置固定更新的周期和前瞻时间
	 */
	public int HRIF_StartServo(int boxID, int rbtID, double dservoTime, double dlookaheadTime)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_StartServo_base(rbtID, dservoTime, dlookaheadTime);
	}

	/**
	 * @Title HRIF_PushServoJ
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dJ1 J1关节坐标
	 * @param dJ2 J2关节坐标
	 * @param dJ3 J3关节坐标
	 * @param dJ4 J4关节坐标
	 * @param dJ5 J5关节坐标
	 * @param dJ6 J6关节坐标
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 在线关节位置命令控制，以 StartServo 设定的固定更新时间发送关节位置，机器人将实时的跟踪
					关节位置指令
	 */
	public int HRIF_PushServoJ(int boxID, int rbtID, double dJ1,double dJ2,double dJ3,double dJ4,double dJ5,double dJ6)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PushServoJ_base(rbtID, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
	}

	/**
	 * @Title HRIF_PushServoP
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
 	 * @param dX 目标空间坐标X
	 * @param dY 目标空间坐标Y
	 * @param dZ 目标空间坐标Z
	 * @param dRx 目标空间坐标Rx
	 * @param dRy 目标空间坐标Ry
	 * @param dRz 目标空间坐标Rz
 	 * @param dTCP_X 工具坐标X
	 * @param dTCP_Y 工具坐标Y
	 * @param dTCP_Z 工具坐标Z
	 * @param dTCP_Rx 工具坐标Rx
	 * @param dTCP_Ry 工具坐标Ry
	 * @param dTCP_Rz 工具坐标Rz
	 * @param dUCS_X 用户坐标X
	 * @param dUCS_Y 用户坐标Y
	 * @param dUCS_Z 用户坐标Z
	 * @param dUCS_Rx 用户坐标Rx
	 * @param dUCS_Ry 用户坐标Ry
	 * @param dUCS_Rz 用户坐标Rz
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 在线末端 TCP位置命令控制，以 StartServo 设定的固定更新时间发送 TCP 位置，机器人将实时
					的跟踪目标 TCP 位置逆运算转换后的关节位置指令
	 */
	public int HRIF_PushServoP(int boxID, int rbtID, double dX, double dY, double dZ,double dRx,double dRy,double dRz,
			double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
			double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PushServoP_base(rbtID, dX, dY, dZ, dRx, dRy, dRz,
				 dTCP_X, dTCP_Y, dTCP_Z, dTCP_Rx, dTCP_Ry, dTCP_Rz,
				 dUCS_X, dUCS_Y, dUCS_Z, dUCS_Rx, dUCS_Ry, dUCS_Rz);
	}

	/**
	 * @Title HRIF_SpeedJ
	 * @date 2024年9月7日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dJ1CmdVel-dJ6CmdVel 关节速度
	 * @param dAcc 加速度
	 * @param dRunTime 运行时长
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 在线关节运动速度伺服控制，以该指令中指定的各个关节的速度和加速度运动指定的时长。
	 */
	public int HRIF_SpeedJ(int boxID, int rbtID,double dJ1CmdVel,double dJ2CmdVel,double dJ3CmdVel,double dJ4CmdVel,
						   double dJ5CmdVel,double dJ6CmdVel,double dAcc,double dRunTime)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SpeedJ_base(rbtID,dJ1CmdVel,dJ2CmdVel,dJ3CmdVel,dJ4CmdVel,dJ5CmdVel,dJ6CmdVel,dAcc,dRunTime);
	}

	/**
	 * @Title HRIF_SpeedL
	 * @param rbtID       机器人ID
	 * @param dXCmdVel
	 * @param dYCmdVel
	 * @param dZCmdVel
	 * @param dRxCmdVel
	 * @param dRyCmdVel
	 * @param dRzCmdVel
	 * @param dLinearAcc
	 * @param dAngularAcc
	 * @param dRunTime    运行时长
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2024年9月7日
	 * @Description 3.12.5.1.在线空间运动速度伺服控制，以该指令中指定的位姿各个坐标的速度和加速度运动指定的时长。
	 */
	public int HRIF_SpeedL(int boxID, int rbtID,double dXCmdVel,double dYCmdVel,double dZCmdVel,double dRxCmdVel,
						   double dRyCmdVel,double dRzCmdVel,double dLinearAcc,double dAngularAcc, double dRunTime)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SpeedL_base(rbtID,dXCmdVel,dYCmdVel,dZCmdVel,dRxCmdVel,dRyCmdVel,dRzCmdVel,dLinearAcc,dAngularAcc,dRunTime);
	}

	/**
	 * @Title HRIF_InitServoEsJ
	 * @date 2022年6月14日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 初始化在线控制模式，清空缓存点位 ,ServoEsJ
	 */
	public int HRIF_InitServoEsJ(int boxID, int rbtID)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_InitServoEsJ_base(rbtID);
	}

	/**
	 * @Title HRIF_StartServoEsJ
	 * @date 2022年6月14日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dServoTime 更新周期
	 * @param dLookaheadTime 前瞻时间
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 启动在线控制模式，设定位置固定更新的周期和前瞻时间，开始运动
	 */
	public int HRIF_StartServoEsJ(int boxID, int rbtID, double dServoTime, double dLookaheadTime)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_StartServoEsJ_base(rbtID, dServoTime, dLookaheadTime);
	}

	/**
	 * @Title HRIF_PushServoEsJ
	 * @date 2022年6月14日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nPointSize 点位数量
	 * @param sPoints 点位信息
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 批量下发在线控制点位 ,每个点位下发频率由固定更新的周期确定
	 */
	public int HRIF_PushServoEsJ(int boxID, int rbtID, int nPointSize, String sPoints)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_PushServoEsJ_base(rbtID, nPointSize, sPoints);
	}

	/**
	 * @Title HRIF_ReadServoEsJState
	 * @date 2022年6月14日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param Ret 点位状态
	 * @param nIndex 当前点位索引
	 * @param nCount 点位数量
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 读取当前是否可以继续下发点位信息，循环读取间隔 >20ms
	 */
	public int HRIF_ReadServoEsJState(int boxID, int rbtID, IntData Ret, IntData nIndex, IntData nCount)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadServoEsJState_base(rbtID, Ret,nIndex,nCount);
	}

	/**-----------------------Part13 相对跟踪运动类控制指令-------------------------*/
	/**
	 * @Title HRIF_SetMoveTraceParams
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nState 跟踪状态
	 * @param dDistance 相对跟踪运动保持的相对距离
	 * @param dAwayVelocity 相对跟踪的运动的探寻速度
	 * @param dBackVelocity 往返速度
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置相对跟踪运动控制参数
	 */
	public int HRIF_SetMoveTraceParams(int boxID, int rbtID, int nState, double dDistance, double dAwayVelocity, double dBackVelocity)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetMoveTraceParams_base(rbtID, nState, dDistance, dAwayVelocity, dBackVelocity);
	}

	/**
	 * @Title HRIF_SetMoveTraceInitParams
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dK 传感器计算参数
	 * @param dB 传感器计算参数
	 * @param dMaxLimit 激光传感器检测距离最大值
	 * @param dMinLimit 激光传感器检测距离最小值
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置相对跟踪运动初始化参数
	 */
	public int HRIF_SetMoveTraceInitParams(int boxID, int rbtID, double dK, double dB, double dMaxLimit, double dMinLimit)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetMoveTraceInitParams_base(rbtID, dK, dB, dMaxLimit, dMinLimit);
	}

	/**
	 * @Title HRIF_SetMoveTraceUcs
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX X
	 * @param dY Y
	 * @param dZ Z
	 * @param dRx Rx
	 * @param dRy Ry
	 * @param dRz Rz
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置相对跟踪运动的跟踪探寻方向
	 */
	public int HRIF_SetMoveTraceUcs(int boxID, int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double dRz)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetMoveTraceUcs_base(rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_SetTrackingState
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nState 跟踪状态
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置传送带跟踪运动状态
	 */
	public int HRIF_SetTrackingState(int boxID, int rbtID, int nState)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetTrackingState_base(rbtID, nState);
	}

	/**-----------------------Part14 位置跟随运动类指令-------------------------*/
	/**
	 * @Title HRIF_SetPoseTrackingMaxMotionLimit
	 * @date 2023年2月27日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dMaxLineVel 直线最大速度
	 * @param dMaxOriVel 姿态最大速度
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置位置跟随的最大跟随速度
	 */
	public int HRIF_SetPoseTrackingMaxMotionLimit(int boxID, int rbtID, double dMaxLineVel, double dMaxOriVel) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetPoseTrackingMaxMotionLimit_base( rbtID, dMaxLineVel, dMaxOriVel);
	}

	/**
	 * @Title HRIF_SetPoseTrackingStopTimeOut
	 * @date 2024年9月11日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dTime 超时时间
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置位置跟踪超时停止时间
	 */
	public int HRIF_SetPoseTrackingStopTimeOut(int boxID, int rbtID, double dTime) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetPoseTrackingStopTimeOut_base( rbtID, dTime);
	}


	/**
	 * @Title HRIF_SetPoseTrackingPIDParams
	 * @date 2023年2月27日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dPosPID1 位置跟随PID
	 * @param dPosPID2 位置跟随PID
	 * @param dPosPID3 位置跟随PID
	 * @param dOriPID1 姿态跟随PID
	 * @param dOriPID2 姿态跟随PID
	 * @param dOriPID3 姿态跟随PID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置PID参数
	 */
	public int HRIF_SetPoseTrackingPIDParams(int boxID, int rbtID, double dPosPID1, double dPosPID2, double dPosPID3, double dOriPID1, double dOriPID2, double dOriPID3) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetPoseTrackingPIDParams_base( rbtID, dPosPID1, dPosPID2, dPosPID3, dOriPID1, dOriPID2, dOriPID3);
	}

	/**
	 * @Title HRIF_SetPoseTrackingTargetPos
	 * @date 2023年2月27日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX X方向保持的距离
	 * @param dY Y方向保持的距离
	 * @param dZ Z方向保持的距离
	 * @param dRx Rx方向保持的距离
	 * @param dRy Ry方向保持的距离
	 * @param dRz Rz方向保持的距离
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置位置跟随的目标位置
	 */
	public int HRIF_SetPoseTrackingTargetPos(int boxID, int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double dRz) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetPoseTrackingTargetPos_base( rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**
	 * @Title HRIF_SetPoseTrackingState
	 * @date 2023年2月27日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nState 位置跟随状态
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置位置跟随状态
	 */
	public int HRIF_SetPoseTrackingState(int boxID, int rbtID, int nState) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetPoseTrackingState_base( rbtID, nState);
	}

	/**
	 * @Title HRIF_SetUpdateTrackingPose
	 * @date 2023年2月27日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param dX 检测到的X方向的距离
	 * @param dY 检测到的Y方向的距离
	 * @param dZ 检测到的Z方向的距离
	 * @param dRx 检测到的Rx方向的距离
	 * @param dRy 检测到的Ry方向的距离
	 * @param dRz 检测到的Rz方向的距离
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置实时更新传感器位置信息
	 */
	public int HRIF_SetUpdateTrackingPose(int boxID, int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double dRz) {
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_SetUpdateTrackingPose_base( rbtID, dX, dY, dZ, dRx, dRy, dRz);
	}

	/**-----------------------Part15 其他指令-------------------------*/

	/**
	 * @Title HRIF_HRAppCmd
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param HRAppName App名称
	 * @param HRAppCmdName 命令名称
	 * @param param 参数列表
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 执行插件 app命令
	 */
	public int HRIF_HRAppCmd(int boxID, String HRAppName, String HRAppCmdName, String param)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_HRAppCmd_base( HRAppName, HRAppCmdName, param);
	}

	/**
	 * @Title HRIF_WriteEndHoldingRegi
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nSlaveID 从站ID
	 * @param nFunction 功能码
	 * @param nRegAddr 寄存器地址
	 * @param nRegCount 寄存器数量
	 * @param VecData 寄存器数据
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 写末端连接的 modbus从站寄存器
	 * @Attention 末端为EtherCAT总线版本IO时有效
	 */
	public int HRIF_WriteEndHoldingRegisters(int boxID, int rbtID, int nSlaveID, int nFunction, int nRegAddr, int nRegCount, Vector<Integer> VecData)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_WriteEndHoldingRegisters_base(rbtID, nSlaveID, nFunction, nRegAddr, nRegCount, VecData);
	}

	/**
	 * @Title HRIF_CopyUserPythonScript_base
	 * @date 2023年3月2日
	 * @param boxID    电箱ID
	 * @param FilePath 文件路径
	 * @param FileName 文件名字
	 */
	public boolean HRIF_CopyUserPythonScript(int boxID,String FilePath, String FileName)
	{
		if(boxID >= MaxBox) {
			return false;
		}
		return m_client[boxID].HRIF_UpLoadFile_base( FilePath,  FileName);

	}

	/**
	 * @Title HRIF_ReadEndHoldingRegisters
	 * @date 2022年6月2日
	 * @param boxID 电箱ID
	 * @param rbtID 机器人ID
	 * @param nSlaveID 从站ID
	 * @param nFunction 功能码
	 * @param nRegAddr 寄存器地址
	 * @param nRegCount 寄存器数量
	 * @param vecData 寄存器数据
	 * @return int, 0:调用成功;>0:返回错误码
 	 * @Description 读末端连接的 modbus从站寄存器，
	 * @Attention 末端为EtherCAT总线版本IO时有效
	 */
	public int HRIF_ReadEndHoldingRegisters(int boxID, int rbtID, int nSlaveID, int nFunction, int nRegAddr, int nRegCount, Vector<Integer> vecData)
	{
		if(boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_ReadEndHoldingRegisters_base(   rbtID,  nSlaveID,  nFunction,  nRegAddr,  nRegCount, vecData);

	}

	/**
	 * @Title HRIF_UpLoadFile
	 * @date 2023年3月2日
	 * @param boxID 电箱ID
	 * @param FilePath 文件路径
	 * @param FileName 文件名字
	 */
//	public boolean HRIF_UpLoadFile(int boxID, String FilePath, String FileName)
//	{
//		if(boxID >= MaxBox) {
//			return false;
//		}
//		return HRIF_UpLoadFile_base(boxID, FilePath,  FileName);
//
//	}

	/**
	 * @Title HRIF_CopyUserPythonScript
	 * @date 2023年3月2日
	 * @param boxID    电箱ID
	 * @param FileName 文件名字
	 */
	public int HRIF_CopyUserPythonScript(int boxID, String FileName) {
		if (boxID >= MaxBox) {
			return ErrorCode.paramsError.value();
		}
		return m_client[boxID].HRIF_CopyUserPythonScript_Base(FileName);

	}
}