package com.hansRobotBaseLib;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.*;
import java.io.File;
import java.io.FileInputStream;

import org.apache.commons.lang3.StringUtils;
import org.apache.commons.net.ftp.FTPClient;
import org.apache.commons.net.ftp.FTPReply;
import java.io.OutputStream;
import java.io.InputStream;

/**
 * @author Hans'Robot
 * @Description Hans'Robot-Elfin Java SDK
 */
public class HansRobotAPI_Base extends VariableType
{
	//文件上传
	private  FTPClient ftp;
	String m_ip="";
	HansSocket m_IFSocket = new HansSocket();
	HansSocket m_IFSocket_FastPort = new HansSocket();
	HansSocket m_IFSocket_PluginPort = new HansSocket();
	public static final String OK = "OK";
	public static final String Fail = "Fail";
	public static int g_nRobotJoints = 6;
	Vector<String> fastCmdList= new Vector<String>();
	List<String> pluginCmdList = new ArrayList<>(Arrays.asList("HRAppCmd"));
//	Thread monitorThread=null;
	boolean isV8=false;

	public void SetAutoTestState(boolean state)
	{
		m_IFSocket.SetAutoTestState(state);
		m_IFSocket_FastPort.SetAutoTestState(state);
		m_IFSocket_PluginPort.SetAutoTestState(state);
	}

	public int ReadRobotType() {
		int nRet = 0;
		int res = 0;
		int rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadRobotType," + rbtID + ",;";
		nRet = SendAndReceive(cmd,rece);
		if (nRet != 0) {
			return nRet;
		}
		if (rece.size() >= 7) {
			int[] AxistType = new int[5];
			AxistType[0] = Integer.parseInt(rece.get(2));
			AxistType[1] = Integer.parseInt(rece.get(3));
			AxistType[2] = Integer.parseInt(rece.get(4));
			AxistType[3] = Integer.parseInt(rece.get(5));
			AxistType[4] = Integer.parseInt(rece.get(6));
			g_nRobotJoints = 5;
			for (int i = 0; i < 5; i++) {
				switch (AxistType[i]) {
					case 1:
					case 2:
					case 4:
						g_nRobotJoints++;
						break;
					default:
						i = 5; // Break out of the loop
						break;
				}
			}
		}
		else
		{
			res = ErrorCode.returnError.value();
		}
		return res;
	}


	private int SendAndReceive( String cmd, Vector<String> receive)
	{
		int commaIndex = cmd.indexOf(',');
		String a = cmd.substring(0, commaIndex);
		if (isV8 && pluginCmdList.contains(a))
		{
			if (m_IFSocket_PluginPort.isConnected())
			{
				synchronized (this)
				{
					return m_IFSocket_PluginPort.doSendAndReceive( cmd, receive);
				}
			}
			else
			{
				System.out.println("Unable to connect to plugin service");
				return ErrorCode.Connect2CPSFailed.value();
			}
		}
		if (fastCmdList.contains(a) && m_IFSocket_FastPort.isConnected())
		{
			synchronized (this)
			{
				return m_IFSocket_FastPort.doSendAndReceive( cmd, receive);
			}
		}
		synchronized (this)
		{
			return m_IFSocket.doSendAndReceive(cmd,receive);
		}
	}
	
	/**	Part1 初始化  */
	/**
	 * @Title HRIF_Connect 
	 * @date 2022年6月1日
	 * @param hostName 控制器IP地址
	 * @param nPort 端口号
	 * @return int,0:调用成功;>0返回错误码
	 * @Description 连接机器人服务器
	 */
	public int HRIF_Connect_base(String hostName, int nPort) {
		int nRet = 0;
		fastCmdList.clear();
		do {
			// 1) 主 socket
			nRet = m_IFSocket.initSocket(hostName, nPort);
			if(nRet!=0)
			{
				break;
			}
			ReadRobotType();
			// 2) plugin 端口
			int pluginPortExist=HRIF_IsHRAppCmdExist_base();
			if(pluginPortExist==0)
			{
				isV8=true;
				m_IFSocket_PluginPort.initSocket(hostName, 40005);
			}

			// 3) fast 端口
			IntData ndFastPort= new IntData() ;
			int nRetFast = HRIF_ReadFastCmdPort_base(0,ndFastPort);
			if(nRetFast==0)
			{
				int nFastPort=ndFastPort.getReturn();
				if (nFastPort != nPort) //如果用户通过快速端口连接就不再额外连接快速端口了
				{
					nRetFast=m_IFSocket_FastPort.initSocket(hostName, nFastPort);
					if(nRetFast==0)
					{
						HRIF_ReadFastCmdList_base(fastCmdList);
					}
				}
			}
		}while(false);
		return nRet;
	}
		/**
	 * @Title HRIF_ReadFastCmdPort_base
	 * @date 2025年6月7日
	 * @param rbtID 机器人ID
	 * @param nPort 工具坐标名称
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前快速端口的返回值
	 */
	public int HRIF_ReadFastCmdPort_base(int rbtID, IntData nPort)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadFastCmdPort,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() != 1)
		{
			return ErrorCode.returnError.value();
		}
		nPort.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}

	public int HRIF_ReadFastCmdList_base(Vector<String> vecFastCmdList)
	{
		int nRet = 0;
		String cmd = "ReadCmdList,;";
		if(m_IFSocket_FastPort.isConnected())
		{
			synchronized (this)
			{
				return m_IFSocket_FastPort.doSendAndReceive( cmd, vecFastCmdList);
			}
		}
		else
		{
			return ErrorCode.isNotConnect.value();
		}

	}
	
	/**
	 * @Title HRIF_DisConnect 
	 * @date 2022年6月2日
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 断开连接机器人服务器
	 */
	public int HRIF_DisConnect_base()
	{
		m_IFSocket.disconnect();
		m_IFSocket_FastPort.disconnect();
		m_IFSocket_PluginPort.disconnect();
		return 0;
	}

	/**
	 * @Title HRIF_IsConnected 
	 * @date 2022年6月1日
	 * @return boolean,true 已连接，false 未连接
	 * @Description 判断控制器是否连接
	 */
	public boolean  HRIF_IsConnected_base()
	{
		if(isV8 && !fastCmdList.isEmpty())
		{
			return m_IFSocket.isConnected() && m_IFSocket_FastPort.isConnected() && m_IFSocket_PluginPort.isConnected();
		}
		else if(!fastCmdList.isEmpty())
		{
			return m_IFSocket.isConnected() && m_IFSocket_FastPort.isConnected();
		}
		else
		{
			return m_IFSocket.isConnected();
		}
	}

	/**
	 * @Title HRIF_ShutdownRobot 
	 * @date 2022年6月1日
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 控制器断电 (断开机器人供电，系统关机 )
	 */
	public int HRIF_ShutdownRobot_base()
	{
		Vector<String> rece = new Vector<String>();
		String cmd = "OSCmd,";
		cmd = cmd + 1 + ",;";
		return SendAndReceive(cmd,rece);
	}


	/**
	 * @Title HRIF_Connect2Box
	 * @date 2022年6月1日
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 连接控制器电箱
	 */
	public int HRIF_Connect2Box_base()
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ConnectToBox,;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_Electrify
	 * @date 2022年6月1日
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 机器人上电
	 */
	public int HRIF_Electrify_base()
	{
		Vector<String> rece = new Vector<String>();
		String cmd = "Electrify,;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_Blackout
	 * @date 2022年6月1日
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 机器人断电
	 */
	public int HRIF_Blackout_base()
	{
		Vector<String> rece = new Vector<String>();
		String cmd = "BlackOut,;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_Connect2Controller
	 * @date 2022年6月1日
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 连接控制器，连接过程中会启动主站，初始化从站，配置参数，检查配置，完成后跳转到去使能状态
	 */
	public int HRIF_Connect2Controller_base()
	{
		Vector<String> rece = new Vector<String>();
		String cmd = "StartMaster,;";
		return SendAndReceive( cmd,rece);
	}

//	
	/**
	 * @Title HRIF_IsSimulateRobot
	 * @date 2022年6月1日
	 * @param nSimulateRobot 是否模拟机器人
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description  获取是否为模拟机器人
	 */
	public int HRIF_IsSimulateRobot_base(IntData nSimulateRobot)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "IsSimulation,;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() != 1)
		{
			return ErrorCode.returnError.value();
		}
		nSimulateRobot.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}

	/**
	 * @Title HRIF_IsControllerStarted
	 * @date 2022年6月1日
	 * @param nStarted 是否启动完成
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description  控制器是否启动完成
	 */
	public int HRIF_IsControllerStarted_base(IntData nStarted )
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadControllerState,;";
			nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() != 1)
		{
			return ErrorCode.returnError.value();
		}
		nStarted.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}

	/**
	 * @Title HRIF_SetSimulation
	 * @date 2024年9月23日
	 * @param nSimulation
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description  设置机器人的模拟状态
	 */
	public int HRIF_SetSimulation_base(int Robot,int nSimulation)
	{
		Vector<String> rece = new Vector<String>();
		String cmd = "SetSimulation,";
		cmd = cmd + Robot + "," + nSimulation + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_GetErrorCodeStr
	 * @param nErrorCode 错误码
	 * @param strErrorMsg 错误码描述
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description  获取错误码解释
	 */
	public int HRIF_GetErrorCodeStr_base( int nErrorCode, StringData strErrorMsg)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GetErrorCodeStr,";
		cmd = cmd + nErrorCode + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() != 1)
		{
			return ErrorCode.returnError.value();
		}
		strErrorMsg.setReturn(rece.get(0));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadVersion
	 * @date 2022年6月1日
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
	public int HRIF_ReadVersion_base(int rbtID,  StringData strVer, IntData nCPSVersion, IntData nCodesysVersion, IntData nBoxVerMajor,
			IntData nBoxVerMid, IntData nBoxVerMin,IntData nAlgorithmVer,IntData nElfinFirmwareVer, StringData sSoftwareVersion)

	{
		int nRet = 0;
		rbtID = 0;
		String s_Ret = "";
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadVersion,;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet != 0)
		{
			return nRet;
		}
		if(rece.size()<8)
		{
			return ErrorCode.returnError.value();
		}
		for(int i =0;i<rece.size();i++) {
			s_Ret += rece.get(i);
			if(i!=rece.size()-1)
			{
				s_Ret += ".";
			}
		}
		strVer.setReturn(s_Ret);
		nCPSVersion.setReturn(Integer.parseInt(rece.get(0)));
		nCodesysVersion.setReturn(Integer.parseInt(rece.get(1)));
		nBoxVerMajor.setReturn(Integer.parseInt(rece.get(2)));
		nBoxVerMid.setReturn(Integer.parseInt(rece.get(3)));
		nBoxVerMin.setReturn(Integer.parseInt(rece.get(4)));
		nAlgorithmVer.setReturn(Integer.parseInt(rece.get(5)));
		nElfinFirmwareVer.setReturn(Integer.parseInt(rece.get(6)));
		sSoftwareVersion.setReturn(rece.get(7));
		return nRet;
	}


	/**
	 * @Title HRIF_ReadRobotModel
	 * @date 2022年6月1日
	 * @param strModel 机器人类型
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取机器人类型
	 */
	public int HRIF_ReadRobotModel_base(StringData strModel)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadRobotModel,;";
		nRet = SendAndReceive(cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() != 1)
		{
			return ErrorCode.returnError.value();
		}
		strModel.setReturn(rece.get(0));
		return nRet;
	}

	/** Part2 轴组控制接口 */
	
	/**
	 * @Title HRIF_GrpEnable 
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 机器人使能命令
	 */
	public int HRIF_GrpEnable_base(int rbtID)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GrpEnable,";
		cmd = cmd + rbtID + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_GrpDisable
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 机器人去使能命令
	 */
	public int HRIF_GrpDisable_base(int rbtID)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GrpDisable,";
		cmd = cmd + rbtID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_GrpReset
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 机器人复位命令
	 */
	public int HRIF_GrpReset_base(int rbtID)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GrpReset,";
		cmd = cmd + rbtID + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_GrpStop
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 机器人停止运动命令
	 */
	public int HRIF_GrpStop_base(int rbtID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GrpStop,";
		cmd = cmd + rbtID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_GrpInterrupt
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 机器人暂停运动命令
	 */
	public int HRIF_GrpInterrupt_base(int rbtID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GrpInterrupt,";
		cmd = cmd + rbtID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_GrpContinue
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 机器人继续运动命令，机器人处于暂停状态时有效
	 */
	public int HRIF_GrpContinue_base(int rbtID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GrpContinue,";
		cmd = cmd + rbtID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_GrpCloseFreeDriver
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 关闭自由驱动
	 */
	public int HRIF_GrpCloseFreeDriver_base(int rbtID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GrpCloseFreeDriver,";
		cmd = cmd + rbtID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_GrpOpenFreeDriver
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 打开自由驱动
	 */
	public int HRIF_GrpOpenFreeDriver_base(int rbtID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GrpOpenFreeDriver,";
		cmd = cmd + rbtID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_EnterSafeGuard
	 * @date 2024年9月11日
	 * @param rbtID 机器人ID
	 * @param bFlag 状态标识
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 强制进入安全光幕（软急停）
	 */
	public int HRIF_EnterSafeGuard_base(int rbtID, int bFlag)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "EnterSafetyGuard,";
		cmd = cmd + rbtID + "," + bFlag + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_OpenBrake
	 * @date 2024年9月9日
	 * @param rbtID 机器人ID
	 * @param nAxisID 轴ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 松闸
	 */
	public int HRIF_OpenBrake_base(int rbtID, int nAxisID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "OpenBrake,";
		cmd = cmd + rbtID + "," + nAxisID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_CloseBrake
	 * @date 2024年9月9日
	 * @param rbtID 机器人ID
	 * @param nAxisID 轴ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 抱闸
	 */
	public int HRIF_CloseBrake_base(int rbtID, int nAxisID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "CloseBrake,";
		cmd = cmd + rbtID + "," + nAxisID + ",;";
		return SendAndReceive( cmd,rece);
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
	public int HRIF_ReadBrakeStatus_base(int rbtID, IntData stateJ1, IntData stateJ2, IntData stateJ3, IntData stateJ4, IntData stateJ5, IntData stateJ6)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadBrakeStatus,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() != 6)
		{
			return ErrorCode.returnError.value();
		}
		stateJ1.setReturn(Integer.parseInt(rece.get(0)));
		stateJ2.setReturn(Integer.parseInt(rece.get(1)));
		stateJ3.setReturn(Integer.parseInt(rece.get(2)));
		stateJ4.setReturn(Integer.parseInt(rece.get(3)));
		stateJ5.setReturn(Integer.parseInt(rece.get(4)));
		stateJ6.setReturn(Integer.parseInt(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_MoveToSS
	 * @param rbtID   机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2024年9月9日
	 * @Description 移动至安全位置
	 */
	public int HRIF_MoveToSS_base(int rbtID)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MoveToSS,";
		cmd = cmd + rbtID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @param rbtID             机器人ID
	 * @param nThreeStageEnable
	 * @param nThreeStageMode
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Title HRIF_ReadTriStageSwitch
	 * @date 2024年9月24日
	 * @Description 读取三段式按钮的开关以及模式
	 */
	public int HRIF_ReadTriStageSwitch_base( int rbtID, IntData nThreeStageEnable, IntData nThreeStageMode)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadTriStageSwitch,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=2)
		{
			return ErrorCode.returnError.value();
		}
		nThreeStageEnable.setReturn(Integer.parseInt(rece.get(0)));
		nThreeStageMode.setReturn(Integer.parseInt(rece.get(1)));
		return nRet;
	}

	/**
	 * @param rbtID             机器人ID
	 * @param nThreeStageEnable
	 * @param nThreeStageMode
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Title 设置三段式按钮的开关以及模式
	 * @date 2024年9月24日
	 * @Description 读取三段式按钮的开关以及模式
	 */
	public int HRIF_SetTriStageSwitch_base(int rbtID, int nThreeStageEnable, int nThreeStageMode)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetTriStageSwitch,";
		cmd = cmd + rbtID +"," + nThreeStageEnable + "," + nThreeStageMode + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_GetLoadIdentifyResult
	 * @param rbtID   机器人ID
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
	public int HRIF_GetLoadIdentifyResult_base(int rbtID,IntData status,IntData progress,DoubleData mass,DoubleData cx,DoubleData cy,DoubleData cz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GetLoadIdentifyResult,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive(cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=6)
		{
			return ErrorCode.returnError.value();
		}
		status.setReturn(Integer.parseInt(rece.get(0)));
		progress.setReturn(Integer.parseInt(rece.get(1)));
		mass.setReturn(Double.parseDouble(rece.get(2)));
		cx.setReturn(Double.parseDouble(rece.get(3)));
		cy.setReturn(Double.parseDouble(rece.get(4)));
		cz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_LoadIdentify
	 * @param rbtID   机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2025年3月20日
	 * @Description 开始负载辨识结果
	 */
	public int HRIF_LoadIdentify_base(int rbtID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "LoadIdentify,";
		cmd = cmd + rbtID + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_ClearCoControlExpandAxisPos_base
	 * @param rbtID   机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2025年3月21日
	 * @Description 清除联动扩展轴位置(设联动扩展轴当前位置为零点)
	 */
	public int HRIF_ClearCoControlExpandAxisPos_base(int rbtID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ClearCoControlExpandAxisPos,";
		cmd = cmd + rbtID + ",;";
		return SendAndReceive( cmd,rece);
	}




	//---------------------------

	/** Part3 脚本控制指令 */

	/**
	 * @Title HRIF_RunFunc
	 * @date 2022年6月1日
	 * @param strFuncName 函数名称
	 * @param param 参数表
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 运行指定脚本函数
	 */
	public int HRIF_RunFunc_base(String strFuncName, Vector<String> param)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "RunFunc,";
		cmd = cmd + strFuncName + ",";
		if(param != null) {
			if(param.size()>0) {
				for (String s : param) {
					if(s == "")
					{
						s = " ";
					}
					cmd = cmd + s + ",";
				}
				cmd = cmd + ";";
			}else {
				cmd = cmd + " ,;";
			}
		}else {
			cmd = cmd + " ,;";
		}
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_StartScript
	 * @date 2022年6月1日
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 执行脚本 Main函数，调用后执行示教器页面编译好的脚本文件 main函数
	 */
	public int HRIF_StartScript_base()
	{
		Vector<String> rece = new Vector<String>();
		String cmd = "StartScript,;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_StopScript
	 * @date 2022年6月1日
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 停止脚本，调用后停止示教器页面正在执行脚本文件
	 */
	public int HRIF_StopScript_base() {
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "StopScript,;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_PauseScript
	 * @date 2022年6月1日
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 暂停脚本，调用后暂停示教器页面正在执行脚本文件
	 */
	public int HRIF_PauseScript_base() {
		Vector<String> rece = new Vector<String>();
		String cmd = "PauseScript,;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_ContinueScript
	 * @date 2022年6月1日
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 继续脚本，调用后继续运行示教器页面正在暂停的脚本文件，不处于暂停状态则返回 20018错误
	 */
	public int HRIF_ContinueScript_base() {
		Vector<String> rece = new Vector<String>();
		String cmd = "ContinueScript,;";
		return SendAndReceive( cmd,rece);
	}

	/** 
	 * @param nRbtID
	 * @param sScriptName
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Title HRIF_SwitchScript
	 * @date 2024年10月12日
	 * @Description 应用指定的脚本程序。
	 */
	public int HRIF_SwitchScript_base( int nRbtID, String sScriptName) {
		Vector<String> rece = new Vector<String>();
		String cmd = "SwitchScript," + nRbtID + "," + sScriptName + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_ReadDefaultScript
	 * @date 2024年10月12日
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 读取系统当前应用的脚本程序的名字。
	 */
	public int HRIF_ReadDefaultScript_base( int nRbtID, StringData sScriptName) {
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadDefaultScript," + nRbtID + ",;";
		int nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		sScriptName.setReturn(rece.get(0));
		return nRet;
	}

	/** Part4 电箱控制指令*/

	/**
	 * @param rbtID           机器人ID
	 * @param nConnected      电箱连接状态
	 * @param n48V_ON         48V电压状态
	 * @param db48OUT_Voltag  48V输出电压值
	 * @param db48OUT_Current 48V输出电流值
	 * @param nRemoteBTN      远程急停状态
	 * @param nThreeStageBTN  三段按钮状态
	 * @param db12V_Voltag
	 * @param db12V_Current
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Title HRIF_ReadBoxInfo
	 * @date 2022年6月1日
	 * @Description 读取电箱信息
	 */
	@SuppressWarnings("unchecked")
	public int HRIF_ReadBoxInfo_base(int rbtID, IntData nConnected, IntData n48V_ON, DoubleData db48OUT_Voltag,
									 DoubleData db48OUT_Current, IntData nRemoteBTN, IntData nThreeStageBTN, DoubleData db12V_Voltag, DoubleData db12V_Current)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadBoxInfo," + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <6)
		{
			return ErrorCode.returnError.value();
		}
		nConnected.setReturn(Integer.parseInt(rece.get(0)));
		n48V_ON.setReturn(Integer.parseInt(rece.get(1)));
		db48OUT_Voltag.setReturn(Double.parseDouble(rece.get(2)));
		db48OUT_Current.setReturn(Double.parseDouble(rece.get(3)));
		nRemoteBTN.setReturn(Integer.parseInt(rece.get(4)));
		nThreeStageBTN.setReturn(Integer.parseInt(rece.get(5)));
		if(rece.size()==8){
			db12V_Voltag.setReturn(Double.parseDouble(rece.get(6)));
			db12V_Current.setReturn(Double.parseDouble(rece.get(7)));
		}
		return nRet;
	}

	/**
	 * @Title HRIF_ReadBoxCI
	 * @date 2022年6月1日
	 * @param nBit 控制数字输入位
	 * @param nVal 控制数字输入对应位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取电箱控制数字输入状态
	 */
	public int HRIF_ReadBoxCI_base(int nBit, IntData nVal) {
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadBoxCI,";
		cmd = cmd + nBit + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		nVal.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}


	/**
	 * @Title HRIF_ReadBoxDI
	 * @date 2022年6月1日
	 * @param nBit 通用数字输入位
	 * @param nVal 控制数字输入对应位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取电箱通用数字输入状态
	 */
	public int HRIF_ReadBoxDI_base(int nBit, IntData nVal) {
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadBoxDI,";
		cmd = cmd + nBit + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		nVal.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}


	/**
	 * @Title HRIF_ReadBoxCO
	 * @date 2022年6月1日
	 * @param nBit 控制数字输出位
	 * @param nVal 控制数字输入对应位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取电箱控制数字输出状态
	 */
	public int HRIF_ReadBoxCO_base(int nBit, IntData nVal) {
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadBoxCO,";
		cmd = cmd + nBit + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		nVal.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}


	/**
	 * @Title HRIF_ReadBoxDO
	 * @date 2022年6月1日
	 * @param nBit 控制数字输出位
	 * @param nVal 控制数字输入对应位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取电箱通用数字输出状态
	 */
	public int HRIF_ReadBoxDO_base(int nBit, IntData nVal) {
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadBoxDO,";
		cmd = cmd + nBit + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		nVal.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}


	/**
	 * @Title HRIF_ReadBoxAI
	 * @date 2022年6月1日
	 * @param nBit 模拟量输入通道
	 * @param dVal 对应的模拟量通道值
	 * @return int,0:调用成功;>0:返回错误码
	 * @Description 读取电箱模拟量输入值
	 */
	public int HRIF_ReadBoxAI_base(int nBit, DoubleData dVal) {
		int nRet = 0;
		double dRet = 0.0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadBoxAI,";
		cmd = cmd + nBit + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		dVal.setReturn(Double.parseDouble(rece.get(0)));
		return nRet;
	}



	/**
	 * @Title HRIF_ReadBoxAO
	 * @date 2022年6月1日
	 * @param nBit 模拟量输出通道
	 * @param nMode 对应的模拟量通道模式
	 * @param dVal 对应的模拟量通道值
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取电箱模拟量输出值
	 */
	public int HRIF_ReadBoxAO_base(int nBit, IntData nMode, DoubleData dVal) {
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadBoxAO,";
		cmd = cmd + nBit + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=2)
		{
			return ErrorCode.returnError.value();
		}
		nMode.setReturn(Integer.parseInt(rece.get(0)));
		dVal.setReturn(Double.parseDouble(rece.get(1)));
		return nRet;
	}

	/**
	 * @Title HRIF_SetBoxCO
	 * @date 2022年6月1日
	 * @param nBit 控制数字输出位
	 * @param nVal 控制数字输出状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置电箱控制数字输出状态
	 */
	public int HRIF_SetBoxCO_base(int nBit, int nVal) {
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetBoxCO,";
		cmd = cmd + nBit + "," + nVal + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_SetBoxDO
	 * @date 2022年6月1日
	 * @param nBit 通用数字输出位
	 * @param nVal 通用数字输出状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置电箱通用数字输出状态
	 */
	public int HRIF_SetBoxDO_base(int nBit, int nVal) {
		Vector<String> rece = new Vector<String>();
		String cmd = "SetBoxDO,";
		cmd = cmd + nBit + "," + nVal + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_SetBoxAOMode
	 * @date 2022年6月1日
	 * @param nBit 模拟量通道
	 * @param nVal 模式
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置电箱模拟量输出模式
	 */
	public int HRIF_SetBoxAOMode_base(int nBit, int nVal) {
		Vector<String> rece = new Vector<String>();
		String cmd = "SetBoxAOMode" +",";
		cmd = cmd + nBit + "," + nVal + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetBoxAOVal
	 * @date 2022年6月1日
	 * @param nBit 模拟量通道
	 * @param dVal 对应模拟量值
	 * @param nMode 模式
	 * @return int,0:调用成功;>0:返回错误码
	 * @Description 设置电箱模拟量输出值
	 */
	public int HRIF_SetBoxAOVal_base(int nBit, double dVal, int nMode) {
		Vector<String> rece = new Vector<String>();
		String cmd = "SetBoxAO,";
		cmd = cmd + nBit + "," + dVal + ","+ nMode +",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_SetEndDO
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param nBit 末端数字输出位
	 * @param nVal 末端数字输出状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置末端数字输出状态
	 */
	public int HRIF_SetEndDO_base(int rbtID, int nBit, int nVal) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetEndDO,";
		cmd = cmd + rbtID + "," + nBit + "," + nVal + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_ReadEndDI
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param nBit 末端数字输入位
	 * @param nVal 末端数字输入对应位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取末端数字输入状态
	 */
	public int HRIF_ReadEndDI_base(int rbtID, int nBit, IntData nVal) {
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadEI,";
		cmd = cmd + rbtID + "," + nBit + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		nVal.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}

	/**
	 * @Title HRIF_SetMoveParamsAO
	 * @param rbtID      机器人ID
	 * @param nState 通道开关状态
	 * @param nIndex 电压模拟量通道
	 * @param dInitAO 初始化电压
	 * @param dWeldingAO 焊接电压
	 * @return int, 0 调用成功;>0 返回错误码
	 * @date 2024年9月10日
	 * @Description 读取末端数字输入状态
	 */
	public int HRIF_SetMoveParamsAO_base( int rbtID, int nState, int nIndex, double dInitAO, double dWeldingAO) {
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = String.format("SetMoveParamsAO,%d,%d,%d", rbtID,nState,nIndex);
		cmd =cmd + "," + dInitAO + "," + dWeldingAO + ",;";
		return SendAndReceive(cmd,rece);
	}



	/**
	 * @Title HRIF_ReadEndDO
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param nBit 末端数字输出位
	 * @param nVal 末端数字输出对应位状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取末端数字输出状态
	 */
	public int HRIF_ReadEndDO_base(int rbtID, int nBit, IntData nVal) {
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadEO,";
		cmd = cmd + rbtID + "," + nBit + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		nVal.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}


	/**
	 * @Title HRIF_ReadEndAI
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param nBit 模拟量输入通道
	 * @param dVal 对应的模拟量通道值
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取末端模拟量输入值
	 */
	public int HRIF_ReadEndAI_base(int rbtID, int nBit, DoubleData dVal) {
		int nRet = 0;
		double nRet_d = 0.0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadEAI,";
		cmd = cmd + rbtID + "," + nBit + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		dVal.setReturn(Double.parseDouble(rece.get(0)));
		return nRet;
	}

	/**
	 * @param nEndDOMask     需要更改的EndDO按bit标识
	 * @param nEndDOVal      各个需要更改的EndDO的目标状态
	 * @param nBoxDOMask     需要更改的BoxDO按bit标识
	 * @param nBoxDOVal      各个需要更改的BoxDO的目标状态
	 * @param nBoxCOMask     需要更改的BoxCO按bit标识
	 * @param nBoxCOVal      各个需要更改的BoxCO的目标状态
	 * @param nBoxAOCH0_Mask BoxAOCH0是否需要更改的标识
	 * @param nBoxAOCH0_Mode 模式
	 * @param nBoxAOCH1_Mask BoxAOCH1是否需要更改的标识
	 * @param dbBoxAOCH0_Val 对应模拟量值
	 * @param dbBoxAOCH1_Val 对应模拟量值
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Title HRIF_cdsSetIO
	 * @date 2024年9月5日
	 * @Description 在运动指令到达目标点位前设置IO。
	 */
	public int HRIF_cdsSetIO_base( int nEndDOMask,int nEndDOVal,int nBoxDOMask,int nBoxDOVal,
								  int nBoxCOMask, int nBoxCOVal, int nBoxAOCH0_Mask, int nBoxAOCH0_Mode, int nBoxAOCH1_Mask,int nBoxAOCH1_Mode,
								  double dbBoxAOCH0_Val, double dbBoxAOCH1_Val) {
		double nRet_d = 0.0;
		Vector<String> rece = new Vector<String>();
		String cmd = "cdsSetIO,";
		cmd = cmd + nEndDOMask + "," + nEndDOVal + "," + nBoxDOMask + "," + nBoxDOVal + "," + nBoxCOMask + "," +
				nBoxCOVal + "," + nBoxAOCH0_Mask + "," + nBoxAOCH0_Mode + "," + nBoxAOCH1_Mask + "," +
				nBoxAOCH1_Mode + "," + dbBoxAOCH0_Val + "," + dbBoxAOCH1_Val + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetCIConfig
	 * @date 2022年6月1日
	 * @param nFunction 功能类型
	 * @param nBit 控制数字输出位
	 * @param nTriggerType 触发方式
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置CI的功能
	 */
	public int HRIF_SetCIConfig_base(int nBit, int nFunction,int nTriggerType) {
		Vector<String> rece = new Vector<String>();
		String cmd = "SetCIConfig,";
		if (nFunction==-1){
			cmd = cmd + nBit + "," + nFunction + ",;";
		}
		else {
			cmd = cmd + nBit + "," + nFunction + "," + nTriggerType + ",;";
		}
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_ReadCIConfig
	 * @date 2024年9月6日
	 * @param nBit 控制数字输出位
	 * @param nFunction 指定nBit的功能类型
	 * @param nTriggerType 功能的触发方式
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取CI的功能
	 */
	public int HRIF_ReadCIConfig_base(int nBit,IntData nFunction,IntData nTriggerType) {
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCIConfig,";
		cmd = cmd + nBit + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=2)
		{
			return ErrorCode.returnError.value();
		}
		nFunction.setReturn(Integer.parseInt(rece.get(0)));
		nTriggerType.setReturn(Integer.parseInt(rece.get(1)));
		return nRet;
	}

	public int HRIF_ReadAllCIConfig_base(IntData nFunction0,IntData nFunction1,IntData nFunction2,
										 IntData nFunction3,IntData nFunction4,IntData nFunction5,IntData nFunction6,
										 IntData nFunction7) {
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCIConfig,";
		cmd = cmd + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=8)
		{
			return ErrorCode.returnError.value();
		}
		nFunction0.setReturn(Integer.parseInt(rece.get(0)));
		nFunction1.setReturn(Integer.parseInt(rece.get(1)));
		nFunction2.setReturn(Integer.parseInt(rece.get(2)));
		nFunction3.setReturn(Integer.parseInt(rece.get(3)));
		nFunction4.setReturn(Integer.parseInt(rece.get(4)));
		nFunction5.setReturn(Integer.parseInt(rece.get(5)));
		nFunction6.setReturn(Integer.parseInt(rece.get(6)));
		nFunction7.setReturn(Integer.parseInt(rece.get(7)));
		return nRet;
	}

	/**
	 * @Title HRIF_SetCOConfig
	 * @date 2024年9月6日
	 * @param nBit 控制数字输出位
	 * @param nFunction 功能类型
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置CO的功能
	 */
	public int HRIF_SetCOConfig_base(int nBit, int nFunction)
	{
		Vector<String> rece = new Vector<String>();
		String cmd = "SetCOConfig,";
		cmd = cmd + nBit + "," + nFunction + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_ReadCOConfig
	 * @date 2024年9月6日
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取CO的功能
	 */
	public int HRIF_ReadCOConfig_base(IntData nFunction0,IntData nFunction1,IntData nFunction2,
										 IntData nFunction3,IntData nFunction4,IntData nFunction5,IntData nFunction6,
										 IntData nFunction7) {
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCOConfig,";
		cmd = cmd + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=8)
		{
			return ErrorCode.returnError.value();
		}
		nFunction0.setReturn(Integer.parseInt(rece.get(0)));
		nFunction1.setReturn(Integer.parseInt(rece.get(1)));
		nFunction2.setReturn(Integer.parseInt(rece.get(2)));
		nFunction3.setReturn(Integer.parseInt(rece.get(3)));
		nFunction4.setReturn(Integer.parseInt(rece.get(4)));
		nFunction5.setReturn(Integer.parseInt(rece.get(5)));
		nFunction6.setReturn(Integer.parseInt(rece.get(6)));
		nFunction7.setReturn(Integer.parseInt(rece.get(7)));
		return nRet;
	}

	/**
	 * @Title HRIF_EnableEndBTN
	 * @date 2024年9月11日
	 * @param rbtID 控制数字输出位
	 * @param nStatus 模块按钮状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 启用或关闭末端
	 */
	public int HRIF_EnableEndBTN_base(int rbtID, int nStatus)
	{
		int nRet = 0;

		Vector<String> rece = new Vector<String>();
		String cmd = "EnableEndBTN,";
		cmd = cmd + rbtID + "," + nStatus + ",;";
		nRet = SendAndReceive(cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		nRet = Integer.parseInt(rece.get(0));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadEndBTN
	 * @date 2022年6月20日
	 * @param rbtID 机器人ID
	 * @param nBit1 按键1状态
	 * @param nBit2 按键2状态
	 * @param nBit3 按键3状态
	 * @param nBit4  按键4状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取末端按键状态,根据搭载的末端类型,各状态表示含义会有区别
	 */
	public int HRIF_ReadEndBTN_base(int rbtID, IntData nBit1,IntData nBit2,IntData nBit3,IntData nBit4)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadEndBTN,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=4)
		{
			return ErrorCode.returnError.value();
		}
		nBit1.setReturn(Integer.parseInt(rece.get(0)));
		nBit2.setReturn(Integer.parseInt(rece.get(1)));
		nBit3.setReturn(Integer.parseInt(rece.get(2)));
		nBit4.setReturn(Integer.parseInt(rece.get(3)));
		return nRet;
	}

	/**Part5 状态读取与设置指令*/

	/**
	 * @Title HRIF_SetOverride
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param dOverride 速度比
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置速度比
	 */
	public int HRIF_SetOverride_base(int rbtID, double dOverride) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetOverride,";
		cmd = cmd + rbtID + "," + dOverride + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_SetToolMotion
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param nState 状态
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 开启或关闭 Tool坐标系运动模式
	 */
	public int HRIF_SetToolMotion_base(int rbtID, int nState) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetToolMotion,";
		cmd = cmd + rbtID + "," + nState + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetPlayload
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param dMass 质量
	 * @param dX 质心X方向偏移
	 * @param dY 质心Y方向偏移
	 * @param dZ 质心Z方向偏移
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置当前负载参数
	 */
	public int HRIF_SetPayload_base(int rbtID, double dMass, double dX, double dY, double dZ)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetPayload,";
		cmd = cmd + rbtID + ",";
		cmd = cmd + dMass + "," + dX + "," + dY + "," + dZ;
		cmd = cmd + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetJointMaxVel
	 * @date 2022年6月1日
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
	public int HRIF_SetJointMaxVel_base(int rbtID, double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetJointMaxVel,";
		cmd = cmd + rbtID + ",";
		cmd = cmd + dJ1 + "," + dJ2 + "," + dJ3 + ",";
		cmd = cmd + dJ4 + "," + dJ5 + "," + dJ6;
		cmd = cmd + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetJointMaxVel_nj
	 * @date 2024年10月11日
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置关节最大运动速度
	 */
	public int HRIF_SetJointMaxVel_nj_base(int rbtID,JointsDatas joints)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetJointMaxVel,";
		cmd = cmd + rbtID + ",";
		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  joints.joint[i] + ",";
		}
		cmd = cmd + ";";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetJointMaxAcc
	 * @date 2022年6月1日
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
	public int HRIF_SetJointMaxAcc_base(int rbtID, double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetJointMaxAcc,";
		cmd = cmd + rbtID + ",";
		cmd = cmd + dJ1 + "," + dJ2 + "," + dJ3 + ",";
		cmd = cmd + dJ4 + "," + dJ5 + "," + dJ6;
		cmd = cmd + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetJointMaxAcc_nj
	 * @date 2024年10月11日
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置关节最大运动加速度
	 */
	public int HRIF_SetJointMaxAcc_nj_base(int rbtID,JointsDatas joints)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetJointMaxAcc,";
		cmd = cmd + rbtID + ",";
		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  joints.joint[i] + ",";
		}
		cmd = cmd + ";";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetMaxAcsRange_nj
	 * @date 2024年10月11日
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置关节最大运动范围
	 */
	public int HRIF_SetMaxAcsRange_nj_base(int rbtID,JointsDatas jointsmax,JointsDatas jointsmin)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetJointMaxAcc,";
		cmd = cmd + rbtID + ",";
		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  jointsmax.joint[i] + ",";
		}
		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  jointsmin.joint[i] + ",";
		}
		cmd = cmd + ";";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetLinearMaxVel
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param dMaxVel 最大直线速度，默认[500], 单位[mm/s]
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置直线运动最大速度
	 */
	public int HRIF_SetLinearMaxVel_base(int rbtID, double dMaxVel)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetLinearMaxVel," + rbtID +"," +dMaxVel+",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetLinearMaxAcc
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param dMaxAcc 最大直线加速度，默认[2500], 单位[mm/s^2]
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置直线运动最大加速度
	 */
	public int HRIF_SetLinearMaxAcc_base(int rbtID, double dMaxAcc)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetLinearMaxAcc," + rbtID +"," +dMaxAcc+",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_SetMaxAcsRange
	 * @date 2022年6月1日
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
	 * @Attention 控制器重启后失效，需要重新设置
	 */
	public int HRIF_SetMaxAcsRange_base(int rbtID, double dMaxJ1, double dMaxJ2,double dMaxJ3,double dMaxJ4,
			double dMaxJ5,double dMaxJ6,double dMinJ1,double dMinJ2,double dMinJ3,double dMinJ4,double dMinJ5,double dMinJ6)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetMaxAcsRange,";
		cmd = cmd + rbtID + "," + dMaxJ1 + "," + dMaxJ2 + ","
				+ dMaxJ3 + "," + dMaxJ4 + "," + dMaxJ5 + ","
				+ dMaxJ6 + ",";
		cmd = cmd + dMinJ1 + "," + dMinJ2 + ","
				+ dMinJ3 + "," + dMinJ4 + "," + dMinJ5 + ","
				+ dMinJ6 + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetMaxPcsRange
	 * @date 2022年6月1日
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
	public int HRIF_SetMaxPcsRange_base(int rbtID, double dMaxX, double dMaxY, double dMaxZ,double dMaxRx,double dMaxRy,double dMaxRz,
				double dMinX, double dMinY,double dMinZ,double dMinRx,double dMinRy,double dMinRz,
				double dUcs_X,double dUcs_Y,double dUcs_Z,double dUcs_Rx,double dUcs_Ry,double dUcs_Rz) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetMaxPcsRange,";
		cmd = cmd + rbtID + "," + dMaxX + "," + dMaxY + ","
				+ dMaxZ + "," + dMaxRx + "," + dMaxRy + ","
				+ dMaxRz + ",";
		cmd = cmd + dMinX + "," + dMinY + ","
				+ dMinZ + "," + dMinRx + "," + dMinRy + ","
				+ dMinRz + ",";
		cmd = cmd + dUcs_X + "," + dUcs_Y + ","
				+ dUcs_Z + "," + dUcs_Rx + ","
				+ dUcs_Ry + "," + dUcs_Rz + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetCollideLevel
	 * @date 2024年9月5日
	 * @param rbtID 机器人ID
	 * @param nSafeLevel 设置安全风险等级
	 * @return int, 0 调用成功;>0 返回错误码
	 */
	public int HRIF_SetCollideLevel_base(int rbtID, int nSafeLevel)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetCollideLevel,";
		cmd = cmd + rbtID + "," + nSafeLevel +",;";
		return SendAndReceive( cmd, rece);
	}

	/**
	 * @Title HRIF_ReadMaxPayload
	 * @date 2024年9月5日
	 * @param rbtID 机器人ID
	 * @param dbMaxPayload 读取末端最大负载
	 * @return int, 0 调用成功;>0 返回错误码
	 */
	public int HRIF_ReadMaxPayload_base(int rbtID, DoubleData dbMaxPayload)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadMaxPayload,";
		cmd = cmd + rbtID +",;";
		nRet = SendAndReceive( cmd, rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		dbMaxPayload.setReturn(Double.parseDouble(rece.get(0)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadPayload
	 * @date 2024年9月5日
	 * @param rbtID 机器人ID
	 * @param dMass 质量
	 * @param dX 质心X方向偏移
	 * @param dY 质心Y方向偏移
	 * @param dZ 质心Z方向偏移
	 * @return int, 0 调用成功;>0 返回错误码
	 * * @Description 读取末端输入
	 */
	public int HRIF_ReadPayload_base(int rbtID, DoubleData dMass,DoubleData dX,DoubleData dY,DoubleData dZ)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadPayload,";
		cmd = cmd + rbtID +",;";
		nRet = SendAndReceive( cmd, rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=4)
		{
			return ErrorCode.returnError.value();
		}
		dMass.setReturn(Double.parseDouble(rece.get(0)));
		dX.setReturn(Double.parseDouble(rece.get(1)));
		dY.setReturn(Double.parseDouble(rece.get(2)));
		dZ.setReturn(Double.parseDouble(rece.get(3)));
		return nRet;
	}


	/**
	 * @Title HRIF_ReadOverride_base
	 * @date 2023年2月11日
	 * @param rbtID 机器人ID
	 * @param dOverride 速度比返回值
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取速度比
	 */
	public int HRIF_ReadOverride_base(int rbtID, DoubleData dOverride)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadOverride,";
		cmd = cmd + rbtID +",;";
		nRet = SendAndReceive(cmd, rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		dOverride.setReturn(Double.parseDouble(rece.get(0)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadJointMaxVel
	 * @date 2022年6月1日
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
	@SuppressWarnings("unchecked")
	public int HRIF_ReadJointMaxVel_base(int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadJointMaxVel,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=6)
		{
			return ErrorCode.returnError.value();
		}
		dJ1.setReturn(Double.parseDouble(rece.get(0)));
		dJ2.setReturn(Double.parseDouble(rece.get(1)));
		dJ3.setReturn(Double.parseDouble(rece.get(2)));
		dJ4.setReturn(Double.parseDouble(rece.get(3)));
		dJ5.setReturn(Double.parseDouble(rece.get(4)));
		dJ6.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadJointMaxVel_nj
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节最大运动速度
	 */
	public int HRIF_ReadJointMaxVel_nj_base(int rbtID,JointsData joints)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadJointMaxVel,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <g_nRobotJoints)
		{
			return ErrorCode.returnError.value();
		}
		for(int i=0;i<g_nRobotJoints;i++)
		{
			joints.joint[i].setReturn(Double.parseDouble(rece.get(i)));
		}
		return nRet;
	}


	/**
	 * @Title HRIF_ReadJointMaxAcc
	 * @date 2022年6月1日
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
	public int HRIF_ReadJointMaxAcc_base(int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadJointMaxAcc,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <6)
		{
			return ErrorCode.returnError.value();
		}
		dJ1.setReturn(Double.parseDouble(rece.get(0)));
		dJ2.setReturn(Double.parseDouble(rece.get(1)));
		dJ3.setReturn(Double.parseDouble(rece.get(2)));
		dJ4.setReturn(Double.parseDouble(rece.get(3)));
		dJ5.setReturn(Double.parseDouble(rece.get(4)));
		dJ6.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadJointMaxAcc_nj
	 * @date 2024年10月12日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节最大运动加速度
	 */
	public int HRIF_ReadJointMaxAcc_nj_base(int rbtID,JointsData joints)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadJointMaxAcc,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <g_nRobotJoints)
		{
			return ErrorCode.returnError.value();
		}
		for(int i=0;i<g_nRobotJoints;i++)
		{
			joints.joint[i].setReturn(Double.parseDouble(rece.get(i)));
		}
		return nRet;
	}

	/**
	 * @Title HRIF_ReadJointMaxJerk
	 * @date 2022年6月1日
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
	public int HRIF_ReadJointMaxJerk_base(int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadJointMaxJerk,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <6)
		{
			return ErrorCode.returnError.value();
		}
		dJ1.setReturn(Double.parseDouble(rece.get(0)));
		dJ2.setReturn(Double.parseDouble(rece.get(1)));
		dJ3.setReturn(Double.parseDouble(rece.get(2)));
		dJ4.setReturn(Double.parseDouble(rece.get(3)));
		dJ5.setReturn(Double.parseDouble(rece.get(4)));
		dJ6.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadJointMaxJerk_nj
	 * @date 2024年10月12日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节最大运动加加速度
	 */
	public int HRIF_ReadJointMaxJerk_nj_base(int rbtID,JointsData joints)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadJointMaxJerk,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <g_nRobotJoints)
		{
			return ErrorCode.returnError.value();
		}
		for(int i=0;i<g_nRobotJoints;i++)
		{
			joints.joint[i].setReturn(Double.parseDouble(rece.get(i)));
		}
		return nRet;
	}

	/**
	 * @Title HRIF_ReadLinearMaxSpeed
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param dMaxVel 最大直线速度
	 * @param dMaxAcc 最大直线加速度
	 * @param dMaxJerk 最大直线加加速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取直线运动最大速度参数
	 */
	public int HRIF_ReadLinearMaxSpeed_base(int rbtID, DoubleData dMaxVel, DoubleData dMaxAcc, DoubleData dMaxJerk)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadLinearMaxVel,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <3)
		{
			return ErrorCode.returnError.value();
		}
		dMaxVel.setReturn(Double.parseDouble(rece.get(0)));
		dMaxAcc.setReturn(Double.parseDouble(rece.get(1)));
		dMaxJerk.setReturn(Double.parseDouble(rece.get(2)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadEmergencyInfo
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param nESTO_Error 急停错误
	 * @param nESTO 急停信号
	 * @param nSafeGuard_Error 安全光幕错误
	 * @param nSafeGuard 安全光幕信号
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取急停信息
	 */
	@SuppressWarnings("unchecked")
	public int HRIF_ReadEmergencyInfo_base(int rbtID, IntData nESTO_Error, IntData nESTO, IntData nSafeGuard_Error, IntData nSafeGuard)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadEmergencyInfo," + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <4)
		{
			return ErrorCode.returnError.value();
		}
		nESTO_Error.setReturn(Integer.parseInt(rece.get(0)));
		nESTO.setReturn(Integer.parseInt(rece.get(1)));
		nSafeGuard_Error.setReturn(Integer.parseInt(rece.get(2)));
		nSafeGuard.setReturn(Integer.parseInt(rece.get(3)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadRobotState
	 * @date 2022年6月1日
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
	@SuppressWarnings("unchecked")
	public int HRIF_ReadRobotState_base(int rbtID, IntData nMovingState,IntData nEnableState,IntData nErrorState,IntData nErrorCode,
			IntData nErrorAxis,IntData nBreaking,IntData nPause,IntData nEmergencyStop,IntData nSafeGuard,
			IntData nElectrify,IntData nIsConnectToBox, IntData nBlendingDone, IntData nInPos) {
		int nRet = 0;
		rbtID = 0;
		List<Integer> nRets = new ArrayList<Integer>();
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadRobotState,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <13)
		{
			return ErrorCode.returnError.value();
		}
		nMovingState.setReturn(Integer.parseInt(rece.get(0)));
		nEnableState.setReturn(Integer.parseInt(rece.get(1)));
		nErrorState.setReturn(Integer.parseInt(rece.get(2)));
		nErrorCode.setReturn(Integer.parseInt(rece.get(3)));
		nErrorAxis.setReturn(Integer.parseInt(rece.get(4)));
		nBreaking.setReturn(Integer.parseInt(rece.get(5)));
		nPause.setReturn(Integer.parseInt(rece.get(6)));
		nEmergencyStop.setReturn(Integer.parseInt(rece.get(7)));
		nSafeGuard.setReturn(Integer.parseInt(rece.get(8)));
		nElectrify.setReturn(Integer.parseInt(rece.get(9)));
		nIsConnectToBox.setReturn(Integer.parseInt(rece.get(10)));
		nBlendingDone.setReturn(Integer.parseInt(rece.get(11)));
		nInPos.setReturn(Integer.parseInt(rece.get(12)));
		return nRet;
	}


	/**
	 * @Title HRIF_ReadRobotFlags
	 * @date 2022年6月1日
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
	public int HRIF_ReadRobotFlags_base(int rbtID, IntData nMovingState,IntData nEnableState,IntData nErrorState,IntData nErrorCode,
			IntData nErrorAxis,IntData nBreaking,IntData nPause, IntData nBlendingDone) {
		int nRet = 0;
		rbtID = 0;
		List<Integer> nRets = new ArrayList<Integer>();
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadRobotState,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <13)
		{
			return ErrorCode.returnError.value();
		}
		nMovingState.setReturn(Integer.parseInt(rece.get(0)));
		nEnableState.setReturn(Integer.parseInt(rece.get(1)));
		nErrorState.setReturn(Integer.parseInt(rece.get(2)));
		nErrorCode.setReturn(Integer.parseInt(rece.get(3)));
		nErrorAxis.setReturn(Integer.parseInt(rece.get(4)));
		nBreaking.setReturn(Integer.parseInt(rece.get(5)));
		nPause.setReturn(Integer.parseInt(rece.get(6)));
		nBlendingDone.setReturn(Integer.parseInt(rece.get(11)));
		return nRet;

	}

	/**
	 * @Title HRIF_ReadCurWaypointID
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param strCurWaypointID 当前ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取WayPoint当前运动 ID号
	 */
	public int HRIF_ReadCurWaypointID_base(int rbtID, StringData strCurWaypointID)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCurWayPointID," + rbtID + ",;";
		nRet = SendAndReceive(cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if (rece.isEmpty()) {
			strCurWaypointID.setReturn("");
		} else if (rece.size() == 1) {
			strCurWaypointID.setReturn(rece.get(0) == null ? "" : rece.get(0));
		} else {
			return ErrorCode.returnError.value();
		}
		return nRet;
	}

	/**
	 * @Title HRIF_ReadAxisErrorCode
	 * @date 2022年6月1日
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
	public int HRIF_ReadAxisErrorCode_base(int rbtID, IntData nErrorCode, IntData nJ1,IntData nJ2,IntData nJ3,IntData nJ4,IntData nJ5,IntData nJ6)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadAxisErrorCode,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <7)
		{
			return ErrorCode.returnError.value();
		}
		nErrorCode.setReturn(Integer.parseInt(rece.get(0)));
		nJ1.setReturn(Integer.parseInt(rece.get(1)));
		nJ2.setReturn(Integer.parseInt(rece.get(2)));
		nJ3.setReturn(Integer.parseInt(rece.get(3)));
		nJ4.setReturn(Integer.parseInt(rece.get(4)));
		nJ5.setReturn(Integer.parseInt(rece.get(5)));
		nJ6.setReturn(Integer.parseInt(rece.get(6)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadAxisErrorCode_nj
	 * @date 2024年10月12日
	 * @param rbtID 机器人ID
	 * @param nErrorCode 当前错误码
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取错误码
	 */
	public int HRIF_ReadAxisErrorCode_nj_base(int rbtID, IntData nErrorCode, JointsData joints)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadAxisErrorCode,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <g_nRobotJoints+1)
		{
			return ErrorCode.returnError.value();
		}
		nErrorCode.setReturn(Integer.parseInt(rece.get(0)));
		for(int i=0;i<g_nRobotJoints;i++)
		{
			joints.joint[i].setReturn(Double.parseDouble(rece.get(1+i)));
		}
		return nRet;
	}


	/**
	 * @Title HRIF_ReadCurFSM
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param nCurFSM 当前状态机
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前状态机
	 */
	public int HRIF_ReadCurFSM_base(int rbtID, IntData nCurFSM ) {
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCurFSM,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		nCurFSM.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadCurFSMFromCPS
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param nCurFSM 当前状态机
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前状态机
	 */
	public int HRIF_ReadCurFSMFromCPS_base(int rbtID, IntData nCurFSM) {
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCurFSM,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=1)
		{
			return ErrorCode.returnError.value();
		}
		nCurFSM.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadPointByName
	 * @date 2023年2月16日
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
	public int HRIF_ReadPointByName_base( int rbtID, String pointName,DoubleData dJ1, DoubleData dJ2, DoubleData dJ3,
			DoubleData dJ4, DoubleData dJ5, DoubleData dJ6, DoubleData dX, DoubleData dY, DoubleData dZ, DoubleData dRx, DoubleData dRy, DoubleData dRz,
			DoubleData dTcp_X, DoubleData dTcp_Y, DoubleData dTcp_Z, DoubleData dTcp_Rx, DoubleData dTcp_Ry, DoubleData dTcp_Rz,
			DoubleData dUcs_X, DoubleData dUcs_Y, DoubleData dUcs_Z, DoubleData dUcs_Rx, DoubleData dUcs_Ry, DoubleData dUcs_Rz) {
		int nRet = 0;
		rbtID =0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadPointByName,";
		cmd = cmd + rbtID + ","+ pointName +",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=24)
		{
			return ErrorCode.returnError.value();
		}
        dJ1.setReturn(Double.parseDouble(rece.get(0)));
        dJ2.setReturn(Double.parseDouble(rece.get(1)));
        dJ3.setReturn(Double.parseDouble(rece.get(2)));
        dJ4.setReturn(Double.parseDouble(rece.get(3)));
        dJ5.setReturn(Double.parseDouble(rece.get(4)));
        dJ6.setReturn(Double.parseDouble(rece.get(5)));
        dX.setReturn(Double.parseDouble(rece.get(6)));
        dY.setReturn(Double.parseDouble(rece.get(7)));
        dZ.setReturn(Double.parseDouble(rece.get(8)));
        dRx.setReturn(Double.parseDouble(rece.get(9)));
        dRy.setReturn(Double.parseDouble(rece.get(10)));
        dRz.setReturn(Double.parseDouble(rece.get(11)));
        dTcp_X.setReturn(Double.parseDouble(rece.get(12)));
        dTcp_Y.setReturn(Double.parseDouble(rece.get(13)));
        dTcp_Z.setReturn(Double.parseDouble(rece.get(14)));
        dTcp_Rz.setReturn(Double.parseDouble(rece.get(15)));
        dTcp_Ry.setReturn(Double.parseDouble(rece.get(16)));
        dTcp_Rz.setReturn(Double.parseDouble(rece.get(17)));
        dUcs_X.setReturn(Double.parseDouble(rece.get(18)));
        dUcs_Y.setReturn(Double.parseDouble(rece.get(19)));
        dUcs_Z.setReturn(Double.parseDouble(rece.get(20)));
        dUcs_Rx.setReturn(Double.parseDouble(rece.get(21)));
        dUcs_Ry.setReturn(Double.parseDouble(rece.get(22)));
        dUcs_Rz.setReturn(Double.parseDouble(rece.get(23)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadPointByName_nj
	 * @date 2024年10月12日
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
	public int HRIF_ReadPointByName_nj_base( int rbtID, String pointName,JointsData joints, DoubleData dX, DoubleData dY, DoubleData dZ, DoubleData dRx, DoubleData dRy, DoubleData dRz,
										 DoubleData dTcp_X, DoubleData dTcp_Y, DoubleData dTcp_Z, DoubleData dTcp_Rx, DoubleData dTcp_Ry, DoubleData dTcp_Rz,
										 DoubleData dUcs_X, DoubleData dUcs_Y, DoubleData dUcs_Z, DoubleData dUcs_Rx, DoubleData dUcs_Ry, DoubleData dUcs_Rz) {
		int nRet = 0;
		rbtID =0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadPointByName,";
		cmd = cmd + rbtID + ","+ pointName +",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <17+g_nRobotJoints)
		{
			return ErrorCode.returnError.value();
		}
		int i=0;
		for(i=0;i<g_nRobotJoints;i++)
		{
			joints.joint[i].setReturn(Double.parseDouble(rece.get(i)));
		}
		dX.setReturn(Double.parseDouble(rece.get(i++)));
		dY.setReturn(Double.parseDouble(rece.get(i++)));
		dZ.setReturn(Double.parseDouble(rece.get(i++)));
		dRx.setReturn(Double.parseDouble(rece.get(i++)));
		dRy.setReturn(Double.parseDouble(rece.get(i++)));
		dRz.setReturn(Double.parseDouble(rece.get(i++)));
		dTcp_X.setReturn(Double.parseDouble(rece.get(i++)));
		dTcp_Y.setReturn(Double.parseDouble(rece.get(i++)));
		dTcp_Z.setReturn(Double.parseDouble(rece.get(i++)));
		dTcp_Rz.setReturn(Double.parseDouble(rece.get(i++)));
		dTcp_Ry.setReturn(Double.parseDouble(rece.get(i++)));
		dTcp_Rz.setReturn(Double.parseDouble(rece.get(i++)));
		dUcs_X.setReturn(Double.parseDouble(rece.get(i++)));
		dUcs_Y.setReturn(Double.parseDouble(rece.get(i++)));
		dUcs_Z.setReturn(Double.parseDouble(rece.get(i++)));
		dUcs_Rx.setReturn(Double.parseDouble(rece.get(i++)));
		dUcs_Ry.setReturn(Double.parseDouble(rece.get(i++)));
		dUcs_Rz.setReturn(Double.parseDouble(rece.get(i++)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadPointList
	 * @date 2024年9月23日
	 * @param rbtID 机器人ID
	 * @param PointList
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取系统中保存的点位名称列表
	 */
	public int HRIF_ReadPointList_base(int rbtID ,StringData PointList)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "ReadPointList,";
		cmd = cmd + rbtID  + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < rece.size() ; i++)
		{
			sb.append(rece.get(i));
			if(i!=rece.size() -1)
			{
				sb.append(",");
			}
		}
		PointList.setReturn(sb.toString());
		return nRet;
	}
	/** Part6 位置/速度/电流读取指令 */

	/**
	 * @Title HRIF_ReadActPos
	 * @date 2022年6月1日
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
	@SuppressWarnings("unchecked")
	public int HRIF_ReadActPos_base(int rbtID,DoubleData dX, DoubleData dY, DoubleData dZ, DoubleData dRx, DoubleData dRy, DoubleData dRz,
			DoubleData dJ1, DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6,
			DoubleData dTCP_X, DoubleData dTCP_Y,DoubleData dTCP_Z,DoubleData dTCP_Rx,DoubleData dTCP_Ry,DoubleData dTCP_Rz,
			DoubleData dUCS_X, DoubleData dUCS_Y,DoubleData dUCS_Z,DoubleData dUCS_Rx,DoubleData dUCS_Ry,DoubleData dUCS_Rz) {
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadActPos,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <24)
		{
			return ErrorCode.returnError.value();
		}
		dJ1.setReturn(Double.parseDouble(rece.get(0)));
		dJ2.setReturn(Double.parseDouble(rece.get(1)));
		dJ3.setReturn(Double.parseDouble(rece.get(2)));
		dJ4.setReturn(Double.parseDouble(rece.get(3)));
		dJ5.setReturn(Double.parseDouble(rece.get(4)));
		dJ6.setReturn(Double.parseDouble(rece.get(5)));

		dX.setReturn(Double.parseDouble(rece.get(6)));
		dY.setReturn(Double.parseDouble(rece.get(7)));
		dZ.setReturn(Double.parseDouble(rece.get(8)));
		dRx.setReturn(Double.parseDouble(rece.get(9)));
		dRy.setReturn(Double.parseDouble(rece.get(10)));
		dRz.setReturn(Double.parseDouble(rece.get(11)));

		dTCP_X.setReturn(Double.parseDouble(rece.get(12)));
		dTCP_Y.setReturn(Double.parseDouble(rece.get(13)));
		dTCP_Z.setReturn(Double.parseDouble(rece.get(14)));
		dTCP_Rx.setReturn(Double.parseDouble(rece.get(15)));
		dTCP_Ry.setReturn(Double.parseDouble(rece.get(16)));
		dTCP_Rz.setReturn(Double.parseDouble(rece.get(17)));

		dUCS_X.setReturn(Double.parseDouble(rece.get(18)));
		dUCS_Y.setReturn(Double.parseDouble(rece.get(19)));
		dUCS_Z.setReturn(Double.parseDouble(rece.get(20)));
		dUCS_Rx.setReturn(Double.parseDouble(rece.get(21)));
		dUCS_Ry.setReturn(Double.parseDouble(rece.get(22)));
		dUCS_Rz.setReturn(Double.parseDouble(rece.get(23)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadActPos_nj
	 * @date 2022年6月1日
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
	public int HRIF_ReadActPos_nj_base(int rbtID,DoubleData dX, DoubleData dY, DoubleData dZ, DoubleData dRx, DoubleData dRy, DoubleData dRz,
									   JointsData joints,
									DoubleData dTCP_X, DoubleData dTCP_Y,DoubleData dTCP_Z,DoubleData dTCP_Rx,DoubleData dTCP_Ry,DoubleData dTCP_Rz,
									DoubleData dUCS_X, DoubleData dUCS_Y,DoubleData dUCS_Z,DoubleData dUCS_Rx,DoubleData dUCS_Ry,DoubleData dUCS_Rz) {
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadActPos,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <g_nRobotJoints+18)
		{
			return ErrorCode.returnError.value();
		}
		int i=0;
		for(i=0;i<g_nRobotJoints;i++)
		{
			joints.joint[i].setReturn(Double.parseDouble(rece.get(i)));
		}

		dX.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dX
		dY.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dY
		dZ.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dZ
		dRx.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dRx
		dRy.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dRy
		dRz.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dRz

		dTCP_X.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dTCP_X
		dTCP_Y.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dTCP_Y
		dTCP_Z.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dTCP_Z
		dTCP_Rx.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dTCP_Rx
		dTCP_Ry.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dTCP_Ry
		dTCP_Rz.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dTCP_Rz

		dUCS_X.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dUCS_X
		dUCS_Y.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dUCS_Y
		dUCS_Z.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dUCS_Z
		dUCS_Rx.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dUCS_Rx
		dUCS_Ry.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dUCS_Ry
		dUCS_Rz.setReturn(Double.parseDouble(rece.get(i++))); // 获取 dUCS_Rz
		return nRet;
	}

	/**
	 * @Title HRIF_ReadCmdJointPos
	 * @date 2022年6月1日
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
	public int HRIF_ReadCmdJointPos_base(int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCmdPos,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
        if(nRet!=ErrorCode.REC_Succeed.value())
        {
            return nRet;
        }
        if(rece.size() <12)
        {
            return ErrorCode.returnError.value();
        }
		dJ1.setReturn(Double.parseDouble(rece.get(0)));
		dJ2.setReturn(Double.parseDouble(rece.get(1)));
		dJ3.setReturn(Double.parseDouble(rece.get(2)));
		dJ4.setReturn(Double.parseDouble(rece.get(3)));
		dJ5.setReturn(Double.parseDouble(rece.get(4)));
		dJ6.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadCmdJointPos_nj
	 * @date 2024年10月12日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节命令位置
	 */
	public int HRIF_ReadCmdJointPos_nj_base(int rbtID, JointsData joints)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCmdPos,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <g_nRobotJoints)
		{
			return ErrorCode.returnError.value();
		}
		for(int i=0;i<g_nRobotJoints;i++)
		{
			joints.joint[i].setReturn(Double.parseDouble(rece.get(i)));
		}
		return nRet;
	}

	/**
	 * @Title HRIF_ReadActJointPos
	 * @date 2022年6月1日
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
	@SuppressWarnings("unchecked")
	public int HRIF_ReadActJointPos_base(int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadActACS,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <6)
		{
			return ErrorCode.returnError.value();
		}
		dJ1.setReturn(Double.parseDouble(rece.get(0)));
		dJ2.setReturn(Double.parseDouble(rece.get(1)));
		dJ3.setReturn(Double.parseDouble(rece.get(2)));
		dJ4.setReturn(Double.parseDouble(rece.get(3)));
		dJ5.setReturn(Double.parseDouble(rece.get(4)));
		dJ6.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadActJointPos_nj
	 * @date 2024年10月12日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节实际位置
	 */
	public int 	HRIF_ReadActJointPos_nj_base(int rbtID, JointsData joints)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadActACS,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <g_nRobotJoints)
		{
			return ErrorCode.returnError.value();
		}
		for(int i=0;i<g_nRobotJoints;i++)
		{
			joints.joint[i].setReturn(Double.parseDouble(rece.get(i)));
		}
		return nRet;
	}

	/**
	 * @Title HRIF_ReadCmdTcpPos
	 * @date 2022年6月1日
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
	public int HRIF_ReadCmdTcpPos_base(int rbtID, DoubleData dX,DoubleData dY,DoubleData dZ,DoubleData dRx,DoubleData dRy,DoubleData dRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCmdPos,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=12)
		{
			return ErrorCode.returnError.value();
		}
		dX.setReturn(Double.parseDouble(rece.get(6)));
		dY.setReturn(Double.parseDouble(rece.get(7)));
		dZ.setReturn(Double.parseDouble(rece.get(8)));
		dRx.setReturn(Double.parseDouble(rece.get(9)));
		dRy.setReturn(Double.parseDouble(rece.get(10)));
		dRz.setReturn(Double.parseDouble(rece.get(11)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadActTcpPos
	 * @date 2022年6月1日
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
	public int HRIF_ReadActTcpPos_base(int rbtID, DoubleData dX,DoubleData dY,DoubleData dZ,DoubleData dRx,DoubleData dRy,DoubleData dRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadActPCS,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=6)
		{
			return ErrorCode.returnError.value();
		}
		dX.setReturn(Double.parseDouble(rece.get(0)));
		dY.setReturn(Double.parseDouble(rece.get(1)));
		dZ.setReturn(Double.parseDouble(rece.get(2)));
		dRx.setReturn(Double.parseDouble(rece.get(3)));
		dRy.setReturn(Double.parseDouble(rece.get(4)));
		dRz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadCmdJointVel
	 * @date 2022年6月1日
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
	@SuppressWarnings("unchecked")
	public int HRIF_ReadCmdJointVel_base(int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCmdJointVel,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=6)
		{
			return ErrorCode.returnError.value();
		}
		dJ1.setReturn(Double.parseDouble(rece.get(0)));
		dJ2.setReturn(Double.parseDouble(rece.get(1)));
		dJ3.setReturn(Double.parseDouble(rece.get(2)));
		dJ4.setReturn(Double.parseDouble(rece.get(3)));
		dJ5.setReturn(Double.parseDouble(rece.get(4)));
		dJ6.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadCmdJointVel_nj
	 * @date 2024年10月12日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节命令速度
	 */
	public int HRIF_ReadCmdJointVel_nj_base(int rbtID,JointsData joints)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCmdJointVel,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <g_nRobotJoints)
		{
			return ErrorCode.returnError.value();
		}
		for(int i=0;i<g_nRobotJoints;i++)
		{
			joints.joint[i].setReturn(Double.parseDouble(rece.get(i)));
		}
		return nRet;
	}

	/**
	 * @Title HRIF_ReadActJointVel
	 * @date 2022年6月1日
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
	@SuppressWarnings("unchecked")
	public int HRIF_ReadActJointVel_base(int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadActJointVel,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <6)
		{
			return ErrorCode.returnError.value();
		}
		dJ1.setReturn(Double.parseDouble(rece.get(0)));
		dJ2.setReturn(Double.parseDouble(rece.get(1)));
		dJ3.setReturn(Double.parseDouble(rece.get(2)));
		dJ4.setReturn(Double.parseDouble(rece.get(3)));
		dJ5.setReturn(Double.parseDouble(rece.get(4)));
		dJ6.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadActJointVel_nj
	 * @date 2024年10月12日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节实际速度
	 */
	public int HRIF_ReadActJointVel_nj_base(int rbtID,JointsData joints)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadActJointVel,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <g_nRobotJoints)
		{
			return ErrorCode.returnError.value();
		}
		for(int i=0;i<g_nRobotJoints;i++)
		{
			joints.joint[i].setReturn(Double.parseDouble(rece.get(i)));
		}
		return nRet;
	}

	/**
	 * @Title HRIF_ReadCmdTcpVel
	 * @date 2022年6月1日
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
	@SuppressWarnings("unchecked")
	public int HRIF_ReadCmdTcpVel_base(int rbtID, DoubleData dX,DoubleData dY,DoubleData dZ,DoubleData dRx,DoubleData dRy,DoubleData dRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCmdTcpVel,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=6)
		{
			return ErrorCode.returnError.value();
		}
		dX.setReturn(Double.parseDouble(rece.get(0)));
		dY.setReturn(Double.parseDouble(rece.get(1)));
		dZ.setReturn(Double.parseDouble(rece.get(2)));
		dRx.setReturn(Double.parseDouble(rece.get(3)));
		dRy.setReturn(Double.parseDouble(rece.get(4)));
		dRz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadActTcpVel
	 * @date 2022年6月1日
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
	@SuppressWarnings("unchecked")
	public int HRIF_ReadActTcpVel_base(int rbtID, DoubleData dX,DoubleData dY,DoubleData dZ,DoubleData dRx,DoubleData dRy,DoubleData dRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadActTcpVel,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=6)
		{
			return ErrorCode.returnError.value();
		}
		dX.setReturn(Double.parseDouble(rece.get(0)));
		dY.setReturn(Double.parseDouble(rece.get(1)));
		dZ.setReturn(Double.parseDouble(rece.get(2)));
		dRx.setReturn(Double.parseDouble(rece.get(3)));
		dRy.setReturn(Double.parseDouble(rece.get(4)));
		dRz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadCmdJointCur
	 * @date 2022年6月1日
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
	public int HRIF_ReadCmdJointCur_base(int rbtID,DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCmdJointCur,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=6)
		{
			return ErrorCode.returnError.value();
		}
		dJ1.setReturn(Double.parseDouble(rece.get(0)));
		dJ2.setReturn(Double.parseDouble(rece.get(1)));
		dJ3.setReturn(Double.parseDouble(rece.get(2)));
		dJ4.setReturn(Double.parseDouble(rece.get(3)));
		dJ5.setReturn(Double.parseDouble(rece.get(4)));
		dJ6.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadCmdJointCur_nj
	 * @date 2024年10月12日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节命令电流
	 */
	public int HRIF_ReadCmdJointCur_nj_base(int rbtID,JointsData joints)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCmdJointCur,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=g_nRobotJoints)
		{
			return ErrorCode.returnError.value();
		}
		for(int i=0;i<g_nRobotJoints;i++)
		{
			joints.joint[i].setReturn(Double.parseDouble(rece.get(i)));
		}
		return nRet;
	}

	/**
	 * @Title HRIF_ReadActJointCur
	 * @date 2022年6月1日
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
	public int HRIF_ReadActJointCur_base(int rbtID, DoubleData dJ1,DoubleData dJ2,DoubleData dJ3,DoubleData dJ4,DoubleData dJ5,DoubleData dJ6)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadActJointCur,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=6)
		{
			return ErrorCode.returnError.value();
		}
		dJ1.setReturn(Double.parseDouble(rece.get(0)));
		dJ2.setReturn(Double.parseDouble(rece.get(1)));
		dJ3.setReturn(Double.parseDouble(rece.get(2)));
		dJ4.setReturn(Double.parseDouble(rece.get(3)));
		dJ5.setReturn(Double.parseDouble(rece.get(4)));
		dJ6.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadActJointCur_nj
	 * @date 2024年10月12日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取关节实际电流
	 */
	public int HRIF_ReadActJointCur_nj_base(int rbtID,JointsData joints)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadActJointCur,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=g_nRobotJoints)
		{
			return ErrorCode.returnError.value();
		}
		for(int i=0;i<g_nRobotJoints;i++)
		{
			joints.joint[i].setReturn(Double.parseDouble(rece.get(i)));
		}
		return nRet;
	}

	/**
	 * @Title HRIF_ReadTcpVelocity
	 * @date 2022年6月1日
	 * @param rbtID 机器人ID
	 * @param dCmdVel 命令速度
	 * @param dActVel 实际速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取TCP末端速度
	 */
	public int HRIF_ReadTcpVelocity_base(int rbtID, DoubleData dCmdVel, DoubleData dActVel)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadTcpVelocity,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=2)
		{
			return ErrorCode.returnError.value();
		}
		dCmdVel.setReturn(Double.parseDouble(rece.get(0)));
		dActVel.setReturn(Double.parseDouble(rece.get(1)));
		return nRet;
	}

	/**
	 * @Title HRIF_SetRotationVelocityControlMode_base
	 * @date 2025年3月21日
	 * @param rbtID 机器人ID
	 * @param nMode 0 : 受限(默认)模式  1 : 不受限模式
	 * @return int, 0 调用成功;>0 返回错误码
	 */
	public int HRIF_SetRotationVelocityControlMode_base(int rbtID, int nMode)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetRotationVelocityControlMode,";
		cmd = cmd + rbtID + "," + nMode +",;";
		return SendAndReceive( cmd, rece);
	}

	/** Part7 坐标转换计算指令 */

	/**
	 * @Title HRIF_Quaternion2RPY
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_Quaternion2RPY_base(int rbtID, double dQuaW, double dQuaX, double dQuaY, double dQuaZ,
			DoubleData dRx, DoubleData dRy, DoubleData dRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "Quaternion2RPY,";
		cmd = cmd + rbtID + ",";
		cmd = cmd + dQuaW + "," + dQuaX + "," + dQuaY + "," + dQuaZ + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=3)
		{
			return ErrorCode.returnError.value();
		}
		dRx.setReturn(Double.parseDouble(rece.get(0)));
		dRy.setReturn(Double.parseDouble(rece.get(1)));
		dRz.setReturn(Double.parseDouble(rece.get(2)));
		return nRet;
	}

	/**
	 * @Title HRIF_RPY2Quaternion
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_RPY2Quaternion_base(int rbtID, double Rx, double Ry, double Rz,
			DoubleData dQuaW, DoubleData dQuaX,DoubleData dQuaY,DoubleData dQuaZ)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "RPY2Quaternion,";
		cmd = cmd + rbtID + ",";
		cmd = cmd + Rx + "," + Ry + "," + Rz + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=4)
		{
			return ErrorCode.returnError.value();
		}
		dQuaW.setReturn(Double.parseDouble(rece.get(0)));
		dQuaX.setReturn(Double.parseDouble(rece.get(1)));
		dQuaY.setReturn(Double.parseDouble(rece.get(2)));
		dQuaZ.setReturn(Double.parseDouble(rece.get(3)));
		return nRet;
	}

	/**
	 * @Title HRIF_GetInverseKin
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_GetInverseKin_base(int rbtID, double dCoord_X, double dCoord_Y, double dCoord_Z, double dCoord_Rx,double dCoord_Ry, double dCoord_Rz,
			double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
			double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz,
			double dJ1,double dJ2,double dJ3,double dJ4,double dJ5,double dJ6,
			DoubleData dTargetJ1,DoubleData dTargetJ2,DoubleData dTargetJ3,DoubleData dTargetJ4,DoubleData dTargetJ5,DoubleData dTargetJ6)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "PCS2ACS,";
		cmd = cmd + rbtID + "," + dCoord_X + "," + dCoord_Y + ","
				+ dCoord_Z + "," + dCoord_Rx + ","
				+ dCoord_Ry + "," + dCoord_Rz + ",";

		cmd = cmd + dJ1 + "," + dJ2 + ","
				+ dJ3 + "," + dJ4 + ","
				+ dJ5 + "," + dJ6 + ",";

		cmd = cmd + dTCP_X + "," + dTCP_Y + ","
				+ dTCP_Z + "," + dTCP_Rx + ","
				+ dTCP_Ry + "," + dTCP_Rz + ",";

		cmd = cmd + dUCS_X + "," + dUCS_Y + ","
				+ dUCS_Z + "," + dUCS_Rx + ","
				+ dUCS_Ry + "," + dUCS_Rz + ",;";

		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() !=6)
		{
			return ErrorCode.returnError.value();
		}
		dTargetJ1.setReturn(Double.parseDouble(rece.get(0)));
		dTargetJ2.setReturn(Double.parseDouble(rece.get(1)));
		dTargetJ3.setReturn(Double.parseDouble(rece.get(2)));
		dTargetJ4.setReturn(Double.parseDouble(rece.get(3)));
		dTargetJ5.setReturn(Double.parseDouble(rece.get(4)));
		dTargetJ6.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_GetInverseKin_nj
	 * @date 2024年10月12日
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
	public int HRIF_GetInverseKin_nj_base(int rbtID, double dCoord_X, double dCoord_Y, double dCoord_Z, double dCoord_Rx,double dCoord_Ry, double dCoord_Rz,
									   double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
									   double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz,
										  JointsDatas joints , JointsData Target)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "PCS2ACS,";
		cmd = cmd + rbtID + "," + dCoord_X + "," + dCoord_Y + ","
				+ dCoord_Z + "," + dCoord_Rx + ","
				+ dCoord_Ry + "," + dCoord_Rz + ",";

		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  joints.joint[i] + ",";
		}

		cmd = cmd + dTCP_X + "," + dTCP_Y + ","
				+ dTCP_Z + "," + dTCP_Rx + ","
				+ dTCP_Ry + "," + dTCP_Rz + ",";

		cmd = cmd + dUCS_X + "," + dUCS_Y + ","
				+ dUCS_Z + "," + dUCS_Rx + ","
				+ dUCS_Ry + "," + dUCS_Rz + ",;";


		nRet = SendAndReceive( cmd,rece);

		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <g_nRobotJoints)
		{
			return ErrorCode.returnError.value();
		}
		for(int i=0;i<g_nRobotJoints;i++)
		{
			Target.joint[i].setReturn(Double.parseDouble(rece.get(i)));
		}
		return nRet;
	}


	/**
	 * @Title HRIF_GetForwardKin
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_GetForwardKin_base(int  rbtID, double dJ1,double dJ2,double dJ3,double dJ4,double dJ5,double dJ6,
			double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
			double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz,
			DoubleData dTargetX,DoubleData dTargetY,DoubleData dTargetZ,DoubleData dTargetRx,DoubleData dTargetRy,DoubleData dTargetRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ACS2PCS,";
		cmd = cmd + rbtID + "," + dJ1 + "," + dJ2 + "," + dJ3 + "," + dJ4 + ","
				+ dJ5 + "," + dJ6 + ",";

		cmd = cmd + dTCP_X + "," + dTCP_Y + ","
				+ dTCP_Z + "," + dTCP_Rx + ","
				+ dTCP_Ry + "," + dTCP_Rz + ",";

		cmd = cmd + dUCS_X + "," + dUCS_Y + ","
				+ dUCS_Z + "," + dUCS_Rx + ","
				+ dUCS_Ry + "," + dUCS_Rz + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <6)
		{
			return ErrorCode.returnError.value();
		}
		dTargetX.setReturn(Double.parseDouble(rece.get(0)));
		dTargetY.setReturn(Double.parseDouble(rece.get(1)));
		dTargetZ.setReturn(Double.parseDouble(rece.get(2)));
		dTargetRx.setReturn(Double.parseDouble(rece.get(3)));
		dTargetRy.setReturn(Double.parseDouble(rece.get(4)));
		dTargetRz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_GetForwardKin_nj
	 * @date 2024年10月12日
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
	public int HRIF_GetForwardKin_nj_base(int  rbtID,
									   double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
									   double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz, JointsDatas joints,
									   DoubleData dTargetX,DoubleData dTargetY,DoubleData dTargetZ,DoubleData dTargetRx,DoubleData dTargetRy,DoubleData dTargetRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ACS2PCS,";
		cmd = cmd + rbtID + "," ;

		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  joints.joint[i] + ",";
		}

		cmd = cmd + dTCP_X + "," + dTCP_Y + ","
				+ dTCP_Z + "," + dTCP_Rx + ","
				+ dTCP_Ry + "," + dTCP_Rz + ",";

		cmd = cmd + dUCS_X + "," + dUCS_Y + ","
				+ dUCS_Z + "," + dUCS_Rx + ","
				+ dUCS_Ry + "," + dUCS_Rz + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <6)
		{
			return ErrorCode.returnError.value();
		}
		dTargetX.setReturn(Double.parseDouble(rece.get(0)));
		dTargetY.setReturn(Double.parseDouble(rece.get(1)));
		dTargetZ.setReturn(Double.parseDouble(rece.get(2)));
		dTargetRx.setReturn(Double.parseDouble(rece.get(3)));
		dTargetRy.setReturn(Double.parseDouble(rece.get(4)));
		dTargetRz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}


	/**
	 * @Title HRIF_Base2UcsTcp
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_Base2UcsTcp_base(int rbtID, double dCoord_X, double dCoord_Y,double dCoord_Z,double dCoord_Rx,double dCoord_Ry,double dCoord_Rz,
			double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
			double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz,
			DoubleData dTargetX,DoubleData dTargetY,DoubleData dTargetZ,DoubleData dTargetRx,DoubleData dTargetRy,DoubleData dTargetRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "Base2UcsTcp,";
		cmd = cmd + rbtID + "," + dCoord_X + "," + dCoord_Y + "," + dCoord_Z + "," + dCoord_Rx + ","
				+ dCoord_Ry + "," + dCoord_Rz + ",";

		cmd = cmd + dTCP_X + "," + dTCP_Y + "," + dTCP_Z + "," + dTCP_Rx + ","
				+ dTCP_Ry + "," + dTCP_Rz + ",";

		cmd = cmd + dUCS_X + "," + dUCS_Y + "," + dUCS_Z + "," + dUCS_Rx + ","
				+ dUCS_Ry + "," + dUCS_Rz + ",;";


		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() <6)
		{
			return ErrorCode.returnError.value();
		}
		dTargetX.setReturn(Double.parseDouble(rece.get(0)));
		dTargetY.setReturn(Double.parseDouble(rece.get(1)));
		dTargetZ.setReturn(Double.parseDouble(rece.get(2)));
		dTargetRx.setReturn(Double.parseDouble(rece.get(3)));
		dTargetRy.setReturn(Double.parseDouble(rece.get(4)));
		dTargetRz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_UcsTcp2Base
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_UcsTcp2Base_base(int rbtID, double dCoord_X, double dCoord_Y,double dCoord_Z,double dCoord_Rx,double dCoord_Ry,double dCoord_Rz,
			double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
			double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz,
			DoubleData dTargetX,DoubleData dTargetY,DoubleData dTargetZ,DoubleData dTargetRx,DoubleData dTargetRy,DoubleData dTargetRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "UcsTcp2Base,";
		cmd = cmd + rbtID + "," + dCoord_X + "," + dCoord_Y + "," + dCoord_Z + "," + dCoord_Rx + ","
				+ dCoord_Ry + "," + dCoord_Rz + ",";

		cmd = cmd + dTCP_X + "," + dTCP_Y + "," + dTCP_Z + "," + dTCP_Rx + ","
				+ dTCP_Ry + "," + dTCP_Rz + ",";

		cmd = cmd + dUCS_X + "," + dUCS_Y + "," + dUCS_Z + "," + dUCS_Rx + ","
				+ dUCS_Ry + "," + dUCS_Rz + ",;";

		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dTargetX.setReturn(Double.parseDouble(rece.get(0)));
		dTargetY.setReturn(Double.parseDouble(rece.get(1)));
		dTargetZ.setReturn(Double.parseDouble(rece.get(2)));
		dTargetRx.setReturn(Double.parseDouble(rece.get(3)));
		dTargetRy.setReturn(Double.parseDouble(rece.get(4)));
		dTargetRz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}


	/**
	 * @Title HRIF_PoseAdd
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_PoseAdd_base(int rbtID, double dPose1_X, double dPose1_Y,double dPose1_Z,double dPose1_Rx,double dPose1_Ry,double dPose1_Rz,
			double dPose2_X, double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,
			DoubleData dPose3_X, DoubleData dPose3_Y,DoubleData dPose3_Z,DoubleData dPose3_Rx,DoubleData dPose3_Ry,DoubleData dPose3_Rz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "PoseAdd,";
		cmd = cmd + rbtID + "," + dPose1_X + "," + dPose1_Y + "," + dPose1_Z + "," + dPose1_Rx + ","
				+ dPose1_Ry + "," + dPose1_Rz + ",";

		cmd = cmd + dPose2_X + "," + dPose2_Y + "," + dPose2_Z + "," + dPose2_Rx + ","
				+ dPose2_Ry + "," + dPose2_Rz + ",;";

		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dPose3_X.setReturn(Double.parseDouble(rece.get(0)));
		dPose3_Y.setReturn(Double.parseDouble(rece.get(1)));
		dPose3_Z.setReturn(Double.parseDouble(rece.get(2)));
		dPose3_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dPose3_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dPose3_Rz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_PoseSub
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_PoseSub_base(int rbtID, double dPose1_X, double dPose1_Y,double dPose1_Z,double dPose1_Rx,double dPose1_Ry,double dPose1_Rz,
			double dPose2_X, double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,
			DoubleData dPose3_X, DoubleData dPose3_Y,DoubleData dPose3_Z,DoubleData dPose3_Rx,DoubleData dPose3_Ry,DoubleData dPose3_Rz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "PoseSub,";
		cmd = cmd + rbtID + "," + dPose1_X + "," + dPose1_Y + "," + dPose1_Z + "," + dPose1_Rx + ","
				+ dPose1_Ry + "," + dPose1_Rz + ",";

		cmd = cmd + dPose2_X + "," + dPose2_Y + "," + dPose2_Z + "," + dPose2_Rx + ","
				+ dPose2_Ry + "," + dPose2_Rz + ",;";


		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dPose3_X.setReturn(Double.parseDouble(rece.get(0)));
		dPose3_Y.setReturn(Double.parseDouble(rece.get(1)));
		dPose3_Z.setReturn(Double.parseDouble(rece.get(2)));
		dPose3_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dPose3_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dPose3_Rz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_PoseTrans
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_PoseTrans_base(int rbtID, double dPose1_X, double dPose1_Y,double dPose1_Z,double dPose1_Rx,double dPose1_Ry,double dPose1_Rz,
			double dPose2_X, double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,
			DoubleData dPose3_X, DoubleData dPose3_Y,DoubleData dPose3_Z,DoubleData dPose3_Rx,DoubleData dPose3_Ry,DoubleData dPose3_Rz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "PoseTrans,";
		cmd = cmd + rbtID + "," + dPose1_X + "," + dPose1_Y + ","
				+ dPose1_Z + "," + dPose1_Rx + ","
				+ dPose1_Ry + "," + dPose1_Rz + ",";

		cmd = cmd + dPose2_X + "," + dPose2_Y + ","
				+ dPose2_Z + "," + dPose2_Rx + ","
				+ dPose2_Ry + "," + dPose2_Rz + ",;";


		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dPose3_X.setReturn(Double.parseDouble(rece.get(0)));
		dPose3_Y.setReturn(Double.parseDouble(rece.get(1)));
		dPose3_Z.setReturn(Double.parseDouble(rece.get(2)));
		dPose3_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dPose3_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dPose3_Rz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_PoseInverse
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_PoseInverse_base(int rbtID, double dPose1_X, double dPose1_Y,double dPose1_Z,double dPose1_Rx,double dPose1_Ry,double dPose1_Rz,
			DoubleData dPose3_X, DoubleData dPose3_Y,DoubleData dPose3_Z,DoubleData dPose3_Rx,DoubleData dPose3_Ry,DoubleData dPose3_Rz)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "PoseInverse,";
		cmd = cmd + rbtID + "," + dPose1_X + "," + dPose1_Y + ","
				+ dPose1_Z + "," + dPose1_Rx + ","
				+ dPose1_Ry + "," + dPose1_Rz + ",;";


		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dPose3_X.setReturn(Double.parseDouble(rece.get(0)));
		dPose3_Y.setReturn(Double.parseDouble(rece.get(1)));
		dPose3_Z.setReturn(Double.parseDouble(rece.get(2)));
		dPose3_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dPose3_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dPose3_Rz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_PoseDist
	 * @date 2022年6月2日
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
	public int HRIF_PoseDist_base(int rbtID,double dPose1_X, double dPose1_Y,double dPose1_Z,double dPose1_Rx,double dPose1_Ry,double dPose1_Rz,
			double dPose2_X, double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz,
			DoubleData dDistance, DoubleData  dAngle)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "PoseDist,";
		cmd = cmd + rbtID + "," + dPose1_X + "," + dPose1_Y + ","
				+ dPose1_Z + "," + dPose1_Rx + ","
				+ dPose1_Ry + "," + dPose1_Rz + ",";

		cmd = cmd + dPose2_X + "," + dPose2_Y + ","
				+ dPose2_Z + "," + dPose2_Rx + ","
				+ dPose2_Ry + "," + dPose2_Rz + ",;";


		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=2)
		{
			return ErrorCode.returnError.value();
		}
		dDistance.setReturn(Double.parseDouble(rece.get(0)));
		dAngle.setReturn(Double.parseDouble(rece.get(1)));
		return nRet;
	}

	/**
	 * @Title HRIF_PoseInterpolate
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_PoseInterpolate_base(int rbtID, double dPose1_X, double dPose1_Y,double dPose1_Z,double dPose1_Rx,double dPose1_Ry,double dPose1_Rz,
			double dPose2_X, double dPose2_Y,double dPose2_Z,double dPose2_Rx,double dPose2_Ry,double dPose2_Rz, double dAlpha,
			DoubleData dPose3_X, DoubleData dPose3_Y,DoubleData dPose3_Z,DoubleData dPose3_Rx,DoubleData dPose3_Ry,DoubleData dPose3_Rz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "PoseInterpolate,";
		cmd = cmd + rbtID + "," + dPose1_X + "," + dPose1_Y + ","
				+ dPose1_Z + "," + dPose1_Rx + ","
				+ dPose1_Ry + "," + dPose1_Rz + ",";

		cmd = cmd + dPose2_X + "," + dPose2_Y + ","
				+ dPose2_Z + "," + dPose2_Rx + ","
				+ dPose2_Ry + "," + dPose2_Rz + ",";

		cmd = cmd + dAlpha + ",;";

		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dPose3_X.setReturn(Double.parseDouble(rece.get(0)));
		dPose3_Y.setReturn(Double.parseDouble(rece.get(1)));
		dPose3_Z.setReturn(Double.parseDouble(rece.get(2)));
		dPose3_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dPose3_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dPose3_Rz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_PoseDefdFrame
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_PoseDefdFrame_base(int rbtID, double dPose1_X,double dPose1_Y,double dPose1_Z,
			double dPose2_X,double dPose2_Y,double dPose2_Z,
			double dPose3_X,double dPose3_Y,double dPose3_Z,
			double dPose4_X,double dPose4_Y,double dPose4_Z,
			double dPose5_X,double dPose5_Y,double dPose5_Z,
			double dPose6_X,double dPose6_Y,double dPose6_Z,
			DoubleData dUcs_X,DoubleData dUcs_Y,DoubleData dUcs_Z,DoubleData dUcs_Rx,DoubleData dUcs_Ry,DoubleData dUcs_Rz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "PoseDefdFrame,";
		cmd = cmd + rbtID + "," + dPose1_X + "," + dPose1_Y + "," + dPose1_Z + ",";
		cmd = cmd + dPose2_X + "," + dPose2_Y + "," + dPose2_Z + ",";
		cmd = cmd + dPose3_X + "," + dPose3_Y + "," + dPose3_Z + ",";
		cmd = cmd + dPose4_X + "," + dPose4_Y + "," + dPose4_Z + ",";
		cmd = cmd + dPose5_X + "," + dPose5_Y + "," + dPose5_Z + ",";
		cmd = cmd + dPose6_X + "," + dPose6_Y + "," + dPose6_Z + ",;";

		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dUcs_X.setReturn(Double.parseDouble(rece.get(0)));
		dUcs_Y.setReturn(Double.parseDouble(rece.get(1)));
		dUcs_Z.setReturn(Double.parseDouble(rece.get(2)));
		dUcs_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dUcs_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dUcs_Rz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_CalUcsPlane
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
	@SuppressWarnings("unchecked")
	public int HRIF_CalUcsPlane_base(int rbtID, double dPose1_X, double dPose1_Y, double dPose1_Z,
									 double dPose2_X, double dPose2_Y, double dPose2_Z,
									 double dPose3_X, double dPose3_Y, double dPose3_Z,
									 DoubleData dRetPose_X, DoubleData dRetPose_Y, DoubleData dRetPose_Z, DoubleData dRetPose_Rx, DoubleData dRetPose_Ry, DoubleData dRetPose_Rz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "CalUcsPlane," + rbtID + "," + dPose1_X + "," + dPose1_Y + "," + dPose1_Z + "," +
				dPose2_X + "," + dPose2_Y + "," + dPose2_Z + "," +
				dPose3_X + "," + dPose3_Y + "," + dPose3_Z + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dRetPose_X.setReturn(Double.parseDouble(rece.get(0)));
		dRetPose_Y.setReturn(Double.parseDouble(rece.get(1)));
		dRetPose_Z.setReturn(Double.parseDouble(rece.get(2)));
		dRetPose_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dRetPose_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dRetPose_Rz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}


	/**
	 * @param rbtID       机器人ID
	 * @param dPose1_X    空间坐标1-X
	 * @param dPose1_Y    空间坐标1-Y
	 * @param dPose1_Z    空间坐标1-Z
	 * @param dPose1_Rx
	 * @param dPose1_Ry
	 * @param dPose1_Rz
	 * @param dPose2_X    空间坐标2-X
	 * @param dPose2_Y    空间坐标2-Y
	 * @param dPose2_Z    空间坐标2-Z
	 * @param dPose2_Rx
	 * @param dPose2_Ry
	 * @param dPose2_Rz
	 * @param dRetPose_X
	 * @param dRetPose_Y
	 * @param dRetPose_Z
	 * @param dRetPose_Rx
	 * @param dRetPose_Ry
	 * @param dRetPose_Rz
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Title HRIF_CalUcsLine
	 * @date 2024年9月10日
	 * @Description 描述：通过两点直线法计算UCS
	 */
	@SuppressWarnings("unchecked")
	public int HRIF_CalUcsLine_base( int rbtID, double dPose1_X, double dPose1_Y, double dPose1_Z, double dPose1_Rx, double dPose1_Ry, double dPose1_Rz,
									double dPose2_X, double dPose2_Y, double dPose2_Z, double dPose2_Rx, double dPose2_Ry, double dPose2_Rz,
									DoubleData dRetPose_X, DoubleData dRetPose_Y, DoubleData dRetPose_Z, DoubleData dRetPose_Rx, DoubleData dRetPose_Ry, DoubleData dRetPose_Rz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "CalUcsLine," + rbtID + "," + dPose1_X + "," + dPose1_Y + "," + dPose1_Z + "," +
				dPose1_Rx + "," + dPose1_Ry + "," + dPose1_Rz + "," +
				dPose2_X + "," + dPose2_Y + "," + dPose2_Z + "," +
				dPose2_Rx + "," + dPose2_Ry + "," + dPose2_Rz + ",;";
		nRet = SendAndReceive(cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dRetPose_X.setReturn(Double.parseDouble(rece.get(0)));
		dRetPose_Y.setReturn(Double.parseDouble(rece.get(1)));
		dRetPose_Z.setReturn(Double.parseDouble(rece.get(2)));
		dRetPose_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dRetPose_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dRetPose_Rz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @param rbtID       机器人ID
	 * @param dPose1_X    空间坐标1-X
	 * @param dPose1_Y    空间坐标1-Y
	 * @param dPose1_Z    空间坐标1-Z
	 * @param dPose1_Rx
	 * @param dPose1_Ry
	 * @param dPose1_Rz
	 * @param dPose2_X    空间坐标2-X
	 * @param dPose2_Y    空间坐标2-Y
	 * @param dPose2_Z    空间坐标2-Z
	 * @param dPose2_Rx
	 * @param dPose2_Ry
	 * @param dPose2_Rz
	 * @param dPose3_X    空间坐标3-X
	 * @param dPose3_Y    空间坐标3-Y
	 * @param dPose3_Z    空间坐标3-Z
	 * @param dPose3_Rx
	 * @param dPose3_Ry
	 * @param dPose3_Rz
	 * @param dRetPose_X
	 * @param dRetPose_Y
	 * @param dRetPose_Z
	 * @param dRetPose_Rx
	 * @param dRetPose_Ry
	 * @param dRetPose_Rz
	 * @param quality
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Title HRIF_CalTcp3P
	 * @date 2024年9月10日
	 * @Description 描述：通过三点平面法计算TCP
	 */
	@SuppressWarnings("unchecked")
	public int HRIF_CalTcp3P_base( int rbtID, double dPose1_X, double dPose1_Y, double dPose1_Z, double dPose1_Rx, double dPose1_Ry, double dPose1_Rz,
								  double dPose2_X, double dPose2_Y, double dPose2_Z, double dPose2_Rx, double dPose2_Ry, double dPose2_Rz,
								  double dPose3_X, double dPose3_Y, double dPose3_Z, double dPose3_Rx, double dPose3_Ry, double dPose3_Rz,
								  DoubleData dRetPose_X, DoubleData dRetPose_Y, DoubleData dRetPose_Z, DoubleData dRetPose_Rx, DoubleData dRetPose_Ry, DoubleData dRetPose_Rz, IntData quality)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "CalTcp3P," + rbtID + "," + dPose1_X + "," + dPose1_Y + "," + dPose1_Z + "," +
				dPose1_Rx + "," + dPose1_Ry + "," + dPose1_Rz + "," +
				dPose2_X + "," + dPose2_Y + "," + dPose2_Z + "," +
				dPose2_Rx + "," + dPose2_Ry + "," + dPose2_Rz + "," +
				dPose3_X + "," + dPose3_Y + "," + dPose3_Z + "," +
				dPose3_Rx + "," + dPose3_Ry + "," + dPose3_Rz + ",;";

		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=7)
		{
			return ErrorCode.returnError.value();
		}
		dRetPose_X.setReturn(Double.parseDouble(rece.get(0)));
		dRetPose_Y.setReturn(Double.parseDouble(rece.get(1)));
		dRetPose_Z.setReturn(Double.parseDouble(rece.get(2)));
		dRetPose_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dRetPose_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dRetPose_Rz.setReturn(Double.parseDouble(rece.get(5)));
		quality.setReturn(Integer.parseInt(rece.get(6)));
		return nRet;
	}

	/**
	 * @param rbtID         机器人ID
	 * @param dPose1_X      空间坐标1-X
	 * @param dPose1_Y      空间坐标1-Y
	 * @param dPose1_Z      空间坐标1-Z
	 * @param dPose1_Rx
	 * @param dPose1_Ry
	 * @param dPose1_Rz
	 * @param dPose2_X      空间坐标2-X
	 * @param dPose2_Y      空间坐标2-Y
	 * @param dPose2_Z      空间坐标2-Z
	 * @param dPose2_Rx
	 * @param dPose2_Ry
	 * @param dPose2_Rz
	 * @param dPose3_X      空间坐标3-X
	 * @param dPose3_Y      空间坐标3-Y
	 * @param dPose3_Z      空间坐标3-Z
	 * @param dPose3_Rx
	 * @param dPose3_Ry
	 * @param dPose3_Rz
	 * @param dPose4_X
	 * @param dPose4_Y
	 * @param dPose4_Z
	 * @param dPose4_Rx
	 * @param dPose4_Ry
	 * @param dPose4_Rz
	 * @param dRetPose_X
	 * @param dRetPose_Y
	 * @param dRetPose_Z
	 * @param dRetPose_Rx
	 * @param dRetPose_Ry
	 * @param dRetPose_Rz
	 * @param quality
	 * @param errorIndex_P1
	 * @param errorIndex_P2
	 * @param errorIndex_P3
	 * @param errorIndex_P4
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Title HRIF_CalTcp4P
	 * @date 2024年9月10日
	 * @Description 描述：通过四点平面法计算TCP
	 */
	@SuppressWarnings("unchecked")
	public int HRIF_CalTcp4P_base( int rbtID, double dPose1_X, double dPose1_Y, double dPose1_Z, double dPose1_Rx, double dPose1_Ry, double dPose1_Rz,
								  double dPose2_X, double dPose2_Y, double dPose2_Z, double dPose2_Rx, double dPose2_Ry, double dPose2_Rz,
								  double dPose3_X, double dPose3_Y, double dPose3_Z, double dPose3_Rx, double dPose3_Ry, double dPose3_Rz,
								  double dPose4_X, double dPose4_Y, double dPose4_Z, double dPose4_Rx, double dPose4_Ry, double dPose4_Rz,
								  DoubleData dRetPose_X, DoubleData dRetPose_Y, DoubleData dRetPose_Z, DoubleData dRetPose_Rx, DoubleData dRetPose_Ry, DoubleData dRetPose_Rz,
								  IntData quality, IntData errorIndex_P1, IntData errorIndex_P2, IntData errorIndex_P3, IntData errorIndex_P4)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "CalTcp4P," + rbtID + "," + dPose1_X + "," + dPose1_Y + "," + dPose1_Z + "," +
				dPose1_Rx + "," + dPose1_Ry + "," + dPose1_Rz + "," +
				dPose2_X + "," + dPose2_Y + "," + dPose2_Z + "," +
				dPose2_Rx + "," + dPose2_Ry + "," + dPose2_Rz + "," +
				dPose3_X + "," + dPose3_Y + "," + dPose3_Z + "," +
				dPose3_Rx + "," + dPose3_Ry + "," + dPose3_Rz + "," +
				dPose4_X + "," + dPose4_Y + "," + dPose4_Z + "," +
				dPose4_Rx + "," + dPose4_Ry + "," + dPose4_Rz + ",;";

		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=11)
		{
			return ErrorCode.returnError.value();
		}
		dRetPose_X.setReturn(Double.parseDouble(rece.get(0)));
		dRetPose_Y.setReturn(Double.parseDouble(rece.get(1)));
		dRetPose_Z.setReturn(Double.parseDouble(rece.get(2)));
		dRetPose_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dRetPose_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dRetPose_Rz.setReturn(Double.parseDouble(rece.get(5)));
		quality.setReturn(Integer.parseInt(rece.get(6)));
		errorIndex_P1.setReturn(Integer.parseInt(rece.get(7)));
		errorIndex_P2.setReturn(Integer.parseInt(rece.get(8)));
		errorIndex_P3.setReturn(Integer.parseInt(rece.get(9)));
		errorIndex_P4.setReturn(Integer.parseInt(rece.get(10)));
		return nRet;
	}

	/** Part8 工具坐标与用户坐标读写指令 */

	/**
	 * @Title HRIF_SetTCP
	 * @date 2022年6月2日
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
	public int HRIF_SetTCP_base(int rbtID, double dTCP_X,double dTCP_Y,double dTCP_Z,double dTCP_Rx,double dTCP_Ry,double dTCP_Rz)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetCurTCP,";
		cmd = cmd + rbtID + "," + dTCP_X + "," + dTCP_Y + "," + dTCP_Z + ","
				+ dTCP_Rx + "," + dTCP_Ry + "," + dTCP_Rz + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetUCS
	 * @date 2022年6月2日
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
	public int HRIF_SetUCS_base(int rbtID, double dUCS_X,double dUCS_Y,double dUCS_Z,double dUCS_Rx,double dUCS_Ry,double dUCS_Rz)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetCurUCS,";
		cmd = cmd + rbtID + "," + dUCS_X + "," + dUCS_Y + "," + dUCS_Z + ","
				+ dUCS_Rx + "," + dUCS_Ry + "," + dUCS_Rz + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_ReadCurTCP
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_ReadCurTCP_base(int rbtID, DoubleData dTCP_X, DoubleData dTCP_Y, DoubleData dTCP_Z, DoubleData dTCP_Rx, DoubleData dTCP_Ry, DoubleData dTCP_Rz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCurTCP,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dTCP_X.setReturn(Double.parseDouble(rece.get(0)));
		dTCP_Y.setReturn(Double.parseDouble(rece.get(1)));
		dTCP_Z.setReturn(Double.parseDouble(rece.get(2)));
		dTCP_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dTCP_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dTCP_Rz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadCurUCS
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int HRIF_ReadCurUCS_base(int rbtID, DoubleData dUCS_X,DoubleData dUCS_Y,DoubleData dUCS_Z,DoubleData dUCS_Rx,DoubleData dUCS_Ry,DoubleData dUCS_Rz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadCurUCS,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dUCS_X.setReturn(Double.parseDouble(rece.get(0)));
		dUCS_Y.setReturn(Double.parseDouble(rece.get(1)));
		dUCS_Z.setReturn(Double.parseDouble(rece.get(2)));
		dUCS_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dUCS_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dUCS_Rz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_SetTCPByName
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param sTcpName 工具坐标名称
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 通过名称设置工具坐标列表中的值为当前工具坐标，对应名称为示教器配置页面 TCP示教的工具
					名称
	 */
	public int HRIF_SetTCPByName_base(int rbtID, String sTcpName) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetTCPByName,";
		cmd = cmd + rbtID + "," + sTcpName + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_SetUCSByName
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param sUcsName 用户坐标名称
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 通过名称设置用户坐标列表中的值为当前用户坐标，对应名称为示教器配置页面用户坐标示教的
					名称
	 */
	public int HRIF_SetUCSByName_base(int rbtID, String sUcsName) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetUCSByName,";
		cmd = cmd + rbtID + "," + sUcsName + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_ReadTCPByName
	 * @date 2022年6月2日
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
	public int HRIF_ReadTCPByName_base(int rbtID, String sTcpName, DoubleData dTCP_X, DoubleData dTCP_Y, DoubleData dTCP_Z, DoubleData dTCP_Rx, DoubleData dTCP_Ry, DoubleData dTCP_Rz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadTCPByName,";
		cmd = cmd + rbtID +","+ sTcpName + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dTCP_X.setReturn(Double.parseDouble(rece.get(0)));
		dTCP_Y.setReturn(Double.parseDouble(rece.get(1)));
		dTCP_Z.setReturn(Double.parseDouble(rece.get(2)));
		dTCP_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dTCP_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dTCP_Rz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadTCPList
	 * @date 2024年9月23日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取系统中保存的 TCP名称列表
	 */
	public int HRIF_ReadTCPList_base(int rbtID ,StringData TCPList)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "ReadTCPList,";
		cmd = cmd + rbtID  + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.isEmpty())
		{
			return ErrorCode.returnError.value();
		}
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < rece.size(); i++) {
			sb.append(rece.get(i));
			if (i < rece.size() -1) sb.append(",");
		}
		TCPList.setReturn(sb.toString());
		return nRet;
	}

	/**
	 * @Title HRIF_ReadUCSList
	 * @date 2024年9月23日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取系统中保存的 UCS名称列表
	 */
	public int HRIF_ReadUCSList_base(int rbtID ,StringData UCSList)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "ReadUCSList,";
		cmd = cmd + rbtID  + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.isEmpty())
		{
			return ErrorCode.returnError.value();
		}
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < rece.size(); i++) {
			sb.append(rece.get(i));
			if (i < rece.size() - 1) sb.append(",");
		}
		UCSList.setReturn(sb.toString());
		return nRet;
	}

	/**
	 * @Title HRIF_ReadUCSByName
	 * @date 2022年6月2日
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
	public int HRIF_ReadUCSByName_base(int rbtID, String sUcsName,  DoubleData dUCS_X,DoubleData dUCS_Y,DoubleData dUCS_Z,DoubleData dUCS_Rx,DoubleData dUCS_Ry,DoubleData dUCS_Rz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadUCSByName,";
		cmd = cmd + rbtID + "," + sUcsName + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dUCS_X.setReturn(Double.parseDouble(rece.get(0)));
		dUCS_Y.setReturn(Double.parseDouble(rece.get(1)));
		dUCS_Z.setReturn(Double.parseDouble(rece.get(2)));
		dUCS_Rx.setReturn(Double.parseDouble(rece.get(3)));
		dUCS_Ry.setReturn(Double.parseDouble(rece.get(4)));
		dUCS_Rz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ConfigTCP
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
	public int HRIF_ConfigTCP_base( int rbtID, String sTcpName, double dTcp_X, double dTcp_Y, double dTcp_Z, double dTcp_Rx, double dTcp_Ry, double dTcp_Rz)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ConfigTCP," + rbtID + "," + sTcpName + "," +
				dTcp_X + "," + dTcp_Y + "," + dTcp_Z + "," +
				dTcp_Rx + "," + dTcp_Ry + "," + dTcp_Rz + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @param rbtID    机器人ID
	 * @param sTcpName
	 * @param dUcs_X
	 * @param dUcs_Y
	 * @param dUcs_Z
	 * @param dUcs_Rx
	 * @param dUcs_Ry
	 * @param dUcs_Rz
	 * @return int, 0：调用成功;>0返回错误码
	 * @Title HRIF_ConfigUCS
	 * @date 2024年9月7日
	 * @Description 新建指定名称的TCP和值
	 */
	public int HRIF_ConfigUCS_base( int rbtID, String sTcpName, double dUcs_X, double dUcs_Y, double dUcs_Z, double dUcs_Rx, double dUcs_Ry, double dUcs_Rz)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ConfigUCS," + rbtID + "," + sTcpName + "," +
				dUcs_X + "," + dUcs_Y + "," + dUcs_Z + "," +
				dUcs_Rx + "," + dUcs_Ry + "," + dUcs_Rz + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetBaseInstallingAngle
	 * @param rbtID    机器人ID
	 * @param nRotation 设定的机座旋转角度
	 * @param nTilt     设定的机座倾斜角度
	 * @return int, 0：调用成功;>0返回错误码
	 * @date 2025年3月20日
	 * @Description 设定机座安装角度
	 */
	public int HRIF_SetBaseInstallingAngle_base(int rbtID, int nRotation, int nTilt)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetBaseInstallingAngle," + rbtID + "," + nRotation + "," +nTilt + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_GetBaseInstallingAngle
	 * @param rbtID    机器人ID
	 * @param nRotation 设定的机座旋转角度
	 * @param nTilt     设定的机座倾斜角度
	 * @return int, 0：调用成功;>0返回错误码
	 * @date 2025年3月20日
	 * @Description 设定机座安装角度
	 */
	public int HRIF_GetBaseInstallingAngle_base(int rbtID, IntData nRotation, IntData nTilt)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GetBaseInstallingAngle," + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=2)
		{
			return ErrorCode.returnError.value();
		}
		nRotation.setReturn(Integer.parseInt(rece.get(0)));
		nTilt.setReturn(Integer.parseInt(rece.get(1)));
		return nRet;
	}


	/** Part9 力控控制指令 */

	/**
	 * @Title HRIF_SetForceControlState
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param nState 力控状态
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控状态
	 */
	public int HRIF_SetForceControlState_base(int rbtID, int nState)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetForceControlState,";
		cmd = cmd + rbtID + "," + nState + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_ReadForceControlState
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param nState 力控状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前力控状态
	 */
	public int HRIF_ReadForceControlState_base(int rbtID, IntData nState)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadFTControlState,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=1)
		{
			return ErrorCode.returnError.value();
		}
		nState.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}

	/**
	 * @Title HRIF_SetForceToolCoordinateMotion
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param nMode 模式;0：关闭;1：开启
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控坐标系方向为 tool坐标方向模式
	 */
	public int HRIF_SetForceToolCoordinateMotion_base(int rbtID, int nMode)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetForceToolCoordinateMotion,";
		cmd = cmd + rbtID + "," + nMode + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_ForceControlInterrupt
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 暂停力控运动，仅暂停力控功能，不暂停运动和脚本
	 */
	public int HRIF_ForceControlInterrupt_base(int rbtID)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GrpFCInterrupt,";
		cmd = cmd + rbtID+ ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_ForceControlContinue
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 继续力控运动，仅继续力控运动功能，不继续运动和脚本
	 */
	public int HRIF_ForceControlContinue_base(int rbtID)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GrpFCContinue,";
		cmd = cmd + rbtID+ ",;";
		return  SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetForceZero
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @return int  0：调用成功;>0返回错误码
	 * @Description 力控清零,在原有数据的基础上重新标定力传感器
	 */
	public int HRIF_SetForceZero_base(int rbtID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetForceZero,";
		cmd = cmd + rbtID+ ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetMaxSearchVelocities
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param dMaxLinearVelocity 直线速度
	 * @param dMaxAngularVelocity 姿态角速度
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控探寻的最大速度
	 */
	public int HRIF_SetMaxSearchVelocities_base(int rbtID, double dMaxLinearVelocity, double dMaxAngularVelocity)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "HRSetMaxSearchVelocities,";
		cmd = cmd + rbtID+ ",";
		cmd = cmd + dMaxLinearVelocity + "," + dMaxAngularVelocity;
		cmd = cmd + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetControlFreedom
	 * @date 2022年6月2日
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
	public int HRIF_SetControlFreedom_base(int rbtID, int nX, int nY, int nZ, int nRx, int nRy, int  nRz)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "HRSetControlFreedom,";
		cmd = cmd + rbtID+ ",";
		cmd = cmd + nX + "," + nY + "," + nZ + "," + nRx+ "," + nRy + "," + nRz;
		cmd = cmd + ",;";
		return  SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetForceControlStrategy
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param nState 控制策略;各轴探寻自由度开关：0：柔顺模式/1：探寻模式
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控控制策略
	 */
	public int HRIF_SetForceControlStrategy_base(int rbtID, int nState)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "HRSetForceControlStrategy,";
		cmd = cmd + rbtID+ ",";
		cmd = cmd + nState;
		cmd = cmd + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetFreeDrivePositionAndOrientation
	 * @date 2022年6月2日
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
	public int HRIF_SetFreeDrivePositionAndOrientation_base(int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double dRz)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetFTPosition,";
		cmd = cmd + rbtID+ ",";
		cmd = cmd + dX + ","+ dY + ","+ dZ + ",";
		cmd = cmd + dRx + ","+ dRy + ","+ dRz ;
		cmd = cmd + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetPIDControlParams
	 * @date 2022年6月2日
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
	public int HRIF_SetPIDControlParams_base(int rbtID, double dFp, double dFi, double dFd, double dTp, double dTi, double dTd)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "HRSetPIDControlParams,";
		cmd = cmd + rbtID+ ",";
		cmd = cmd + dFp + "," + dFi + "," + dFd + "," + dTp+ "," + dTi + "," + dTd;
		cmd = cmd + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetMassParams
	 * @date 2022年6月2日
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
	public int HRIF_SetMassParams_base(int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double dRz)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "HRSetMassParams,";
		cmd = cmd + rbtID+ ",";
		cmd = cmd + dX + "," + dY + "," + dZ + "," + dRx+ "," + dRy + "," + dRz;
		cmd = cmd + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetDampParams
	 * @date 2022年6月2日
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
	public int HRIF_SetDampParams_base(int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double dRz)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "HRSetDampParams,";
		cmd = cmd + rbtID+ ",";
		cmd = cmd + dX + "," + dY + "," + dZ + "," + dRx+ "," + dRy + "," + dRz;
		cmd = cmd + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetStiffParams
	 * @date 2022年6月2日
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
	public int HRIF_SetStiffParams_base(int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double dRz)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "HRSetStiffParams,";
		cmd = cmd + rbtID+ ",";
		cmd = cmd + dX + "," + dY + "," + dZ + "," + dRx+ "," + dRy + "," + dRz;
		cmd = cmd + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetForceControlGoal
	 * @date 2022年6月2日
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
	public int HRIF_SetForceControlGoal_base(int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double dRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "HRSetControlGoal,";
		cmd = cmd + rbtID+ ",";
		cmd = cmd + dX + "," + dY + "," + dZ + "," + dRx+ "," + dRy + "," + dRz;
		cmd = cmd + ",0,0,0,0,0,0"  +",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetControlGoal
	 * @date 2022年6月2日
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

	public int HRIF_SetControlGoal_base(int rbtID, double dWrench_X, double dWrench_Y, double dWrench_Z, double dWrench_Rx, double dWrench_Ry, double dWrench_Rz,
			double dDistance_X,double dDistance_Y,double dDistance_Z,double dDistance_Rx,double dDistance_Ry,double dDistance_Rz)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		dDistance_X = 0;
		dDistance_Y = 0;
		dDistance_Z = 0;
		dDistance_Rx = 0;
		dDistance_Ry = 0;
		dDistance_Rz = 0;
		String cmd = "HRSetControlGoal,";
		cmd = cmd + rbtID + "," + dWrench_X + "," + dWrench_Y + "," + dWrench_Z + ","
				+ dWrench_Rx + "," + dWrench_Ry + "," + dWrench_Rz + ",";

		cmd = cmd + dDistance_X + "," + dDistance_Y + "," + dDistance_Z + ","
				+ dDistance_Rx + "," + dDistance_Ry + "," + dDistance_Rz + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetForceDataLimit
	 * @date 2022年6月2日
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
	public int HRIF_SetForceDataLimit_base(int rbtID, double dMax_X, double dMax_Y, double dMax_Z, double dMax_Rx, double dMax_Ry, double dMax_Rz,
			double dMin_X,double dMin_Y,double dMin_Z,double dMin_Rx,double dMin_Ry,double dMin_Rz)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "HRSetForceDataLimit,";
		cmd = cmd + rbtID + "," + dMax_X + "," + dMax_Y + "," + dMax_Z + ","
				+ dMax_Rx + "," + dMax_Ry + "," + dMax_Rz + ",";

		cmd = cmd + dMin_X + "," + dMin_Y + "," + dMin_Z + ","
				+ dMin_Rx + "," + dMin_Ry + "," + dMin_Rz + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetForceDistanceLimit
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param dAllowDistance 允许最大距离
	 * @param dStrengthLevel 位置与边界设置偏离距离的幂次项;2：阻力与偏离边界的平方项成比例；设成3：就表示阻力与偏离边界的立方项成比例
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置力控形变范围
	 */
	public int HRIF_SetForceDistanceLimit_base(int rbtID, double dAllowDistance, double dStrengthLevel)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "HRSetForceDistanceLimit,";
		cmd = cmd + rbtID + "," + dAllowDistance + "," + dStrengthLevel + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetForceFreeDriveMode
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param bEnable 是否开启,1:开启 /0:关闭
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 设置开启或者关闭力控自由驱动模式
	 */
	public int HRIF_SetForceFreeDriveMode_base(int rbtID, int bEnable)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetFTFreeDriveState,";
		cmd = cmd + rbtID + "," + bEnable + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_ReadFTCabData
	 * @date 2022年6月2日
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
	@SuppressWarnings("unchecked")
	public int  HRIF_ReadFTCabData_base(int rbtID, DoubleData dX,DoubleData dY,DoubleData dZ,DoubleData dRx,DoubleData dRy,DoubleData dRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadFTCabData" + "," + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		dX.setReturn(Double.parseDouble(rece.get(0)));
		dY.setReturn(Double.parseDouble(rece.get(1)));
		dZ.setReturn(Double.parseDouble(rece.get(2)));
		dRx.setReturn(Double.parseDouble(rece.get(3)));
		dRy.setReturn(Double.parseDouble(rece.get(4)));
		dRz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadFTFreeDriveSpeedMode
	 * @date 2024年9月7日
	 * @param rbtID 机器人ID
	 * @param nMode 速度模式
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取设定后的自由驱动速度模式。
	 */
	public int  HRIF_ReadFTFreeDriveSpeedMode_base(int rbtID,IntData nMode)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadFTFreeDriveSpeedMode" + "," + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=1)
		{
			return ErrorCode.returnError.value();
		}
		nMode.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}

	/**
	 * @Title HRIF_SetFTFreeDriveSpeedMode
	 * @date 2024年9月7日
	 * @param rbtID 机器人ID
	 * @param nMode 速度模式
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置力控自由驱动速度模式
	 */
	public int  HRIF_SetFTFreeDriveSpeedMode_base(int rbtID,int nMode)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetFTFreeDriveSpeedMode" + "," + rbtID + "," + nMode + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetFreeDriveMotionFreedom
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
	public int  HRIF_SetFreeDriveMotionFreedom_base( int rbtID, int nX, int nY, int nZ, int nRx, int nRy, int nRz)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = String.format("SetFTMotionFreedom,%d,%d,%d,%d,%d,%d,%d,;", rbtID, nX, nY, nZ, nRx, nRy, nRz);
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_ReadFTMotionFreedom
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
	public int  HRIF_ReadFTMotionFreedom_base( int rbtID, IntData nX, IntData nY, IntData nZ, IntData nRx, IntData nRy, IntData nRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = String.format("ReadFTMotionFreedom,%d,;", rbtID);
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=6)
		{
			return ErrorCode.returnError.value();
		}
		nX.setReturn(Integer.parseInt(rece.get(0)));
		nY.setReturn(Integer.parseInt(rece.get(1)));
		nZ.setReturn(Integer.parseInt(rece.get(2)));
		nRx.setReturn(Integer.parseInt(rece.get(3)));
		nRy.setReturn(Integer.parseInt(rece.get(4)));
		nRz.setReturn(Integer.parseInt(rece.get(5)));
		return nRet;
	}

	/**
	 * @Title HRIF_SetFTFreeDriveSpeedMode
	 * @param rbtID    机器人ID
	 * @param dLinear 平移柔顺度
	 * @param dAngular 旋转柔顺度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @date 2024年9月7日
	 * @Description 读取标定后的力传感器数据
	 */
	public int  HRIF_SetFTFreeFactor_base( int rbtID, double dLinear, double dAngular)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetFTFreeFactor" + "," + rbtID + "," + dLinear + "," + dAngular + ",;";
		return  SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetFreeDriveCompensateForce
	 * @date 2024年9月10日
	 * @param rbtID 机器人ID
	 * @param dForce 补偿力
	 * @param dX 补偿力的方向
	 * @param dY
	 * @param dZ
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置FreeDrive模式下的定向补偿力大小及矢量方向
	 */
	public int  HRIF_SetFreeDriveCompensateForce_base(int rbtID,double dForce, double dX, double dY, double dZ)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetFreeDriveCompensateForce," + rbtID + "," +
				dForce + "," + dX + "," + dY + "," + dZ + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @param rbtID            机器人ID
	 * @param dForceThreshold  补偿力的方向
	 * @param dTorqueThreshold
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Title HRIF_SetFTWrenchThresholds
	 * @date 2024年9月10日
	 * @Description 设置力控自由驱动启动阈值（力与力矩）
	 */
	public int  HRIF_SetFTWrenchThresholds_base( int rbtID, double dForceThreshold, double dTorqueThreshold)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetFTWrenchThresholds," + rbtID + "," +
				dForceThreshold + "," + dTorqueThreshold + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetMaxFreeDriveVel
	 * @date 2024年9月10日
	 * @param rbtID 机器人ID
	 * @param dMaxLinearVelocity 直线速度
	 * @param dMaxAngularVelocity 姿态角速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置力控自由驱动最大直线速度及姿态角速度
	 */
	public int  HRIF_SetMaxFreeDriveVel_base(int rbtID, double dMaxLinearVelocity, double dMaxAngularVelocity)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetMaxFreeDriveVel," + rbtID + "," +
				dMaxLinearVelocity + "," + dMaxAngularVelocity + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @param rbtID  机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Title HRIF_SetMaxSearchDistance
	 * @date 2024年9月10日
	 * @Description 设置各自由度力控探寻最大距离
	 */
	public int  HRIF_SetMaxSearchDistance_base(int rbtID, double Dis_X, double Dis_Y, double Dis_Z, double Dis_RX, double Dis_RY, double Dis_RZ)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "HRSetMaxSearchDistance," + rbtID + "," +
				Dis_X + "," + Dis_Y + "," + Dis_Z + "," +
				Dis_RX + "," + Dis_RY + "," + Dis_RZ + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @param rbtID  机器人ID
	 * @param pos_X
	 * @param pos_Y
	 * @param pos_Z
	 * @param pos_RX
	 * @param pos_RY
	 * @param pos_RZ
	 * @param neg_X
	 * @param neg_Y
	 * @param neg_Z
	 * @param neg_RX
	 * @param neg_RY
	 * @param neg_RZ
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Title HRIF_SetSteadyContactDeviationRange
	 * @date 2024年9月10日
	 * @Description 设置各自由度力控探寻最大距离
	 */
	public int  HRIF_SetSteadyContactDeviationRange_base( int rbtID, double pos_X, double pos_Y, double pos_Z, double pos_RX, double pos_RY, double pos_RZ, double neg_X, double neg_Y, double neg_Z, double neg_RX, double neg_RY, double neg_RZ)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "HRSetSteadyContactDeviationRange," + rbtID + "," +
				pos_X + "," + pos_Y + "," + pos_Z + "," +
				pos_RX + "," + pos_RY + "," + pos_RZ + "," +
				neg_X + "," + neg_Y + "," + neg_Z + "," +
				neg_RX + "," + neg_RY + "," + neg_RZ + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetTangentForceBounds
	 * @param rbtID    机器人ID
	 * @param dMax 最大值
	 * @param dMin 最小值
	 * @param dVel 抬升速度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @date 2024年9月7日
	 * @Description 设置标定后的力传感器数据
	 */
	public int  HRIF_SetTangentForceBounds_base(int rbtID, double dMax, double dMin, double dVel)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetTangentForceBounds" + "," + rbtID + "," + dMax + "," + dMin + "," + dVel + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @param rbtID   机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Title HRIF_AddSafePlane
	 * @date 2024年9月27日
	 * @Description 添加虚拟墙平面
	 */
	public int  HRIF_AddSafePlane_base( int rbtID, String Name, String UcsName, int mode, int display, int aSwitch)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "AddSafePlane" + "," + rbtID + "," + Name + "," + UcsName + "," + mode + "," + display + "," + aSwitch + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_UpdateSafePlane
	 * @date 2024年9月27日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 修改更新虚拟墙平面属性
	 */
	public int  HRIF_UpdateSafePlane_base(int rbtID, String Name, String UcsName, int mode, int display, int aSwitch)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "UpdateSafePlane" + "," + rbtID + "," + Name + "," + UcsName + "," + mode + "," + display + "," + aSwitch + ",;";
		return  SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_DelSafePlane
	 * @date 2024年9月27日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 删除虚拟墙平面
	 */
	public int  HRIF_DelSafePlane_base(int rbtID, String Name)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "DelSafePlane" + "," + rbtID + "," + Name  + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_ReadSafePlaneList
	 * @date 2024年9月27日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 返回结果为所有安全平面的名字清单
	 */
	public int  HRIF_ReadSafePlaneList_base(int rbtID, StringData BordersName)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadSafePlane" + "," + rbtID  + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < rece.size(); i++) {
			sb.append(rece.get(i));
			if (i < rece.size() - 1) sb.append(",");
		}
		BordersName.setReturn(sb.toString());
		return nRet;
	}

	/**
	 * @Title HRIF_ReadSafePlane
	 * @date 2024年9月27日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 返回结果为指定安全平面的详细参数
	 */
	public int  HRIF_ReadSafePlane_base( int rbtID, String BorderName ,StringData UcsName ,IntData Mode,IntData Display,IntData Switch)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadSafePlane" + "," + rbtID + "," + BorderName  + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=4)
		{
			return ErrorCode.returnError.value();
		}
		UcsName.setReturn(rece.get(0));
		Mode.setReturn(Integer.parseInt(rece.get(1)));
		Display.setReturn(Integer.parseInt(rece.get(2)));
		Switch.setReturn(Integer.parseInt(rece.get(3)));
		return nRet;
	}

	/**
	 * @Title HRIF_SetDepthThresholdForDampingArea
	 * @date 2024年9月27日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置虚拟墙开始产生阻尼时的距离阈值
	 */
	public int  HRIF_SetDepthThresholdForDampingArea_base(int rbtID,double dDepth)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetDepthThresholdForDampingArea" + "," + rbtID + "," + dDepth  + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_GetLastCalibParams
	 * @date 2024年11月30日
	 * @param rbtID 机器人ID
	 * @param Fx0-Fz0 x-z方向标定力
	 * @param Mx0-Mz0 x-z方向力矩
	 * @param X-Z 重心偏移
	 * @param G 重力
	 * @param InstallRotationAngel 安装角度
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 获取力控标定结果
	 */
	public int  HRIF_GetLastCalibParams_base( int rbtID,DoubleData Fx0,DoubleData Fy0,DoubleData Fz0,DoubleData Mx0,DoubleData My0,DoubleData Mz0,DoubleData G,DoubleData X,DoubleData Y,DoubleData Z,DoubleData InstallRotationAngel)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GetLastCalibParams" + "," + rbtID  + ",;";
			nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()<11)
		{
			return ErrorCode.returnError.value();
		}
		Fx0.setReturn(Double.parseDouble(rece.get(0)));
		Fy0.setReturn(Double.parseDouble(rece.get(1)));
		Fz0.setReturn(Double.parseDouble(rece.get(2)));
		Mx0.setReturn(Double.parseDouble(rece.get(3)));
		My0.setReturn(Double.parseDouble(rece.get(4)));
		Mz0.setReturn(Double.parseDouble(rece.get(5)));
		G.setReturn(Double.parseDouble(rece.get(6)));
		X.setReturn(Double.parseDouble(rece.get(7)));
		Y.setReturn(Double.parseDouble(rece.get(8)));
		Z.setReturn(Double.parseDouble(rece.get(9)));
		InstallRotationAngel.setReturn(Double.parseDouble(rece.get(10)));
		return nRet;
	}

	/**
	 * @Title HRIF_SetInitializeForceSensor
	 * @date 2024年11月30日
	 * @param rbtID 机器人ID
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 设置力控标定结果
	 */
	public int  HRIF_SetInitializeForceSensor_base(int rbtID,double Fx0,double Fy0,double Fz0,double Mx0,double My0,double Mz0,double G,double X,double Y,double Z,double InstallRotationAngel)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetInitializeForceSensor" + "," + rbtID  + "," + Fx0 + "," + Fy0 + "," + Fz0 + "," + Mx0 + "," + My0 + "," + Mz0 + "," + G + "," + X + "," + Y + "," + Z + "," + InstallRotationAngel  + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetFTCalibration_base
	 * @date 2025年3月20日
	 * @param rbtID 机器人ID
	 * @param Pointnum 标定点位数量
	 * @param Points 标定点位置信息
	 * @param forces 标定点力信息
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 计算力控标定数据，接收 8~16 组标定数据，每组数据包含 12 个参数（6 个位置参数 + 6 个力控参数）
	 */
	public int  HRIF_SetFTCalibration_base( int rbtID,int Pointnum,Vector<Position> Points,Vector<Position> forces)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetFTCalibration" + "," + rbtID  + "," ;
		for (int i = 0; i < Pointnum; i++)
		{
			cmd=cmd+Points.get(i).dx+"," ;
			cmd=cmd+Points.get(i).dy+"," ;
			cmd=cmd+Points.get(i).dz+"," ;
			cmd=cmd+Points.get(i).dRx+"," ;
			cmd=cmd+Points.get(i).dRy+"," ;
			cmd=cmd+Points.get(i).dRz+"," ;
			cmd=cmd+forces.get(i).dx+"," ;
			cmd=cmd+forces.get(i).dy+"," ;
			cmd=cmd+forces.get(i).dz+"," ;
			cmd=cmd+forces.get(i).dRx+"," ;
			cmd=cmd+forces.get(i).dRy+"," ;
			cmd=cmd+forces.get(i).dRz+"," ;
		}
		cmd=cmd+";";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_ReadForceData
	 * @date 2025年3月20日
	 * @param rbtID 机器人ID
	 * @param dx-dRz x-Rz方向原始力数据
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取力传感器原始数据
	 */
	public int  HRIF_ReadForceData_base(int rbtID,DoubleData dx,DoubleData dy,DoubleData dz,DoubleData dRx,DoubleData dRy,DoubleData dRz)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadForceData" + "," + rbtID  + ",;";
		nRet = SendAndReceive(cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()<6)
		{
			return ErrorCode.returnError.value();
		}
		dx.setReturn(Double.parseDouble(rece.get(0)));
		dy.setReturn(Double.parseDouble(rece.get(1)));
		dz.setReturn(Double.parseDouble(rece.get(2)));
		dRx.setReturn(Double.parseDouble(rece.get(3)));
		dRy.setReturn(Double.parseDouble(rece.get(4)));
		dRz.setReturn(Double.parseDouble(rece.get(5)));
		return nRet;
	}

	/** Part10 通用运动类控制指令 */

	/**
	 * @Title HRIF_ShortJogJ
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param naxisId 关节轴ID
	 * @param nderection 运动方向
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 关节短点动 运动距离2度,最大速度10度/s
	 */
	public int HRIF_ShortJogJ_base(int rbtID, int naxisId, int nderection) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ShortJogJ,";
		cmd = cmd + rbtID + "," + naxisId + "," + nderection + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_ShortJogL
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param naxisId 坐标系轴ID
	 * @param nderection 方向
	 * @return int, 0:调用成功;>0返回错误码
	 * @Description 空间坐标短点动 运动距离2mm，最大速度10mm/s
	 */
	public int HRIF_ShortJogL_base(int rbtID, int naxisId, int nderection) {
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ShortJogL,";
		cmd = cmd + rbtID + "," + naxisId + "," + nderection + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_LongJogJ
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param naxisId 轴ID
	 * @param nderection 方向
	 * @param nstate 状态
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 关节长点动，最大运动速度10/s
	 * @Attention 必须要与HRIF_LongMoveEvent()配合使用
	 */
	public int HRIF_LongJogJ_base(int rbtID, int naxisId, int nderection, int nstate) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "LongJogJ,";
		cmd = cmd + rbtID + "," + naxisId + "," + nderection + "," + nstate + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_LongJogL
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param naxisId 轴ID
	 * @param nderection 方向
	 * @param nstate 状态
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 空间长点动,最大运动速度50mm/s
	 * @Attention 必须要与HRIF_LongMoveEvent()配合使用
	 */
	public int HRIF_LongJogL_base(int rbtID, int naxisId, int nderection, int nstate) {
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "LongJogL,";
		cmd = cmd + rbtID + "," + naxisId + "," + nderection + "," + nstate + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_LongMoveEvent
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 长点动继续指令，当开始长点动之后，要按 500 毫秒或更短时间为时间周期发送一次该指令，否
则长点动会停止
	 */
	public int HRIF_LongMoveEvent_base(int rbtID) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "LongMoveEvent,";
		cmd = cmd + rbtID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_IsMotionDone
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param bDone 是否处于运动状态
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 判断机器人是否处于运动状态
	 */
	public int HRIF_IsMotionDone_base(int rbtID, BooleanData bDone)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "ReadRobotState,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=13)
		{
			return ErrorCode.returnError.value();
		}
		if(Integer.parseInt(rece.get(1)) == 0 ||
				Integer.parseInt(rece.get(2)) == 1 ||
				Integer.parseInt(rece.get(7)) == 1 ||
				Integer.parseInt(rece.get(8)) == 1 ||
				Integer.parseInt(rece.get(9)) == 0 ||
				Integer.parseInt(rece.get(10)) == 0)
		{
			nRet = ErrorCode.StateRefuse.value();
			return nRet;
		}
		bDone.setReturn(Integer.parseInt(rece.get(0))== 0 && Integer.parseInt(rece.get(11)) == 1);
		return nRet;
	}

	/**
	 * @Title HRIF_IsBlendingDone
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param nDone 路点运动是否完成
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 判断路点是否运动完成
	 */
	public int HRIF_IsBlendingDone_base(int rbtID, BooleanData nDone)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		int nRet = 0;
		String cmd = "ReadRobotState,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive(cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=13)
		{
			return ErrorCode.returnError.value();
		}
		if(Integer.parseInt(rece.get(1)) == 0 ||
				Integer.parseInt(rece.get(2)) == 1 ||
				Integer.parseInt(rece.get(7)) == 1 ||
				Integer.parseInt(rece.get(8)) == 1 ||
				Integer.parseInt(rece.get(9)) == 0 ||
				Integer.parseInt(rece.get(10)) == 0
		) {
			nRet = ErrorCode.StateRefuse.value();
			return nRet;
		}
		nDone.setReturn(Integer.parseInt(rece.get(11)) == 1);
		return nRet;
	}

	/**
	 * @Title HRIF_WayPointEx
	 * @date 2022年6月9日
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
	public int HRIF_WayPointEx_base(int rbtID, int moveType,
			double dX, double dY, double dZ, double dRx, double dRy, double dRz,
			double dJ1, double dJ2,double dJ3,double dJ4,double dJ5,double dJ6,
			double dTCP_X, double dTCP_Y,double dTCP_Z,double dTCP_Rx,double dTCP_Ry,double dTCP_Rz,
			double dUCS_X, double dUCS_Y,double dUCS_Z,double dUCS_Rx,double dUCS_Ry,double dUCS_Rz,
			double dvelocity, double dacceleration, double dradius, int nisUseJoint, int nisSeek, int nIOBit, int nIOState, String strCmdID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "WayPointEx,";
		cmd = cmd + rbtID + "," + dX + "," + dY + "," + dZ + "," + dRx + ","
				+ dRy + "," + dRz + ",";
		cmd += dJ1 + "," + dJ2 + "," + dJ3 + "," + dJ4 + ","
				+ dJ5 + "," + dJ6 + ",";
		cmd += dUCS_X + "," + dUCS_Y + "," + dUCS_Z + "," + dUCS_Rx + ","
				+ dUCS_Ry + "," + dUCS_Rz + ",";
		cmd += dTCP_X + "," + dTCP_Y + "," + dTCP_Z + "," + dTCP_Rx + ","
				+ dTCP_Ry + "," + dTCP_Rz + ",";
		cmd += dvelocity + "," + dacceleration + "," + dradius + ",";
		cmd += moveType + "," + nisUseJoint + "," + nisSeek + "," + nIOBit + "," + nIOState + "," + strCmdID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_WayPointEx_nj
	 * @date 2024年10月12日
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
	public int HRIF_WayPointEx_nj_base(int rbtID, int moveType,
									double dX, double dY, double dZ, double dRx, double dRy, double dRz,
									JointsDatas joints,
									double dTCP_X, double dTCP_Y,double dTCP_Z,double dTCP_Rx,double dTCP_Ry,double dTCP_Rz,
									double dUCS_X, double dUCS_Y,double dUCS_Z,double dUCS_Rx,double dUCS_Ry,double dUCS_Rz,
									double dvelocity, double dacceleration, double dradius, int nisUseJoint, int nisSeek, int nIOBit, int nIOState, String strCmdID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "WayPointEx,";
		cmd = cmd + rbtID + "," + dX + "," + dY + "," + dZ + "," + dRx + ","
				+ dRy + "," + dRz + ",";
		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  joints.joint[i] + ",";
		}
		cmd += dUCS_X + "," + dUCS_Y + "," + dUCS_Z + "," + dUCS_Rx + ","
				+ dUCS_Ry + "," + dUCS_Rz + ",";
		cmd += dTCP_X + "," + dTCP_Y + "," + dTCP_Z + "," + dTCP_Rx + ","
				+ dTCP_Ry + "," + dTCP_Rz + ",";
		cmd += dvelocity + "," + dacceleration + "," + dradius + ",";
		cmd += moveType + "," + nisUseJoint + "," + nisSeek + "," + nIOBit + "," + nIOState + "," + strCmdID + ",;";
		return SendAndReceive(cmd,rece);
	}


	/**
	 * @Title HRIF_WayPoint
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param moveType 运动类型,0：关节运动;1：直线运动
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
	public int HRIF_WayPoint_base(int rbtID, int moveType,
			double dX, double dY, double dZ, double dRx, double dRy, double dRz,
			double dJ1, double dJ2,double dJ3,double dJ4,double dJ5,double dJ6,
			String stcpName, String sucsName,
			double dvelocity, double dacceleration, double dradius, int nisUseJoint, int nisSeek, int nbit,
			int nstate, String strCmdID) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "WayPoint,";
		cmd = cmd + rbtID + "," + dX + "," + dY + "," + dZ + "," + dRx + ","
				+ dRy + "," + dRz + ",";
		cmd = cmd + dJ1 + "," + dJ2 + "," + dJ3 + "," + dJ4 + ","
				+ dJ5 + "," + dJ6 + ",";
		cmd = cmd + stcpName + "," + sucsName + "," + dvelocity + ","
				+ dacceleration + "," + dradius + ",";
		cmd = cmd + moveType + "," + nisUseJoint + "," + nisSeek + "," + nbit + "," + nstate + "," + strCmdID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_WayPoint_nj
	 * @date 2024年10月12日
	 * @param rbtID 机器人ID
	 * @param moveType 运动类型,0：关节运动;1：直线运动
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
	public int HRIF_WayPoint_nj_base(int rbtID, int moveType,
								  double dX, double dY, double dZ, double dRx, double dRy, double dRz,
								  JointsDatas joints,
								  String stcpName, String sucsName,
								  double dvelocity, double dacceleration, double dradius, int nisUseJoint, int nisSeek, int nbit,
								  int nstate, String strCmdID) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "WayPoint,";
		cmd = cmd + rbtID + "," + dX + "," + dY + "," + dZ + "," + dRx + ","
				+ dRy + "," + dRz + ",";
		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  joints.joint[i] + ",";
		}
		cmd = cmd + stcpName + "," + sucsName + "," + dvelocity + ","
				+ dacceleration + "," + dradius + ",";
		cmd = cmd + moveType + "," + nisUseJoint + "," + nisSeek + "," + nbit + "," + nstate + "," + strCmdID + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_WayPoint2
	 * @date 2022年6月10日
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
	public int HRIF_WayPoint2_base(int rbtID, int nmoveType,
			double dEndPose_X, double dEndPose_Y, double dEndPose_Z, double dEndPose_Rx, double dEndPose_Ry, double dEndPose_Rz,
			double dAuxPose_X, double dAuxPose_Y, double dAuxPose_Z, double dAuxPose_Rx, double dAuxPose_Ry, double dAuxPose_Rz,
			double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6,
			String sTcpName, String sUcsName,
			double dvelocity, double dacceleration, double dradius, int nisUseJoint, int nisSeek, int nIOBit, int nIOState, String strCmdID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "WayPoint2,";
		cmd = cmd + rbtID + "," + dEndPose_X + "," + dEndPose_Y + "," + dEndPose_Z + ","
				+ dEndPose_Rx + "," + dEndPose_Ry + "," + dEndPose_Rz + ",";

		cmd = cmd + dJ1 + "," + dJ2 + "," + dJ3 + "," + dJ4 + ","
				+ dJ5 + "," + dJ6 + ",";

		cmd = cmd + sTcpName + "," + sUcsName + "," + dvelocity + ","
				+ dacceleration + "," + dradius + ",";

		cmd = cmd + nmoveType + "," + nisUseJoint + "," + nisSeek + ","
				+ nIOBit + "," + nIOState + ",";

		cmd = cmd + dAuxPose_X + "," + dAuxPose_Y + "," + dAuxPose_Z + ","
				+ dAuxPose_Rx + "," + dAuxPose_Ry + "," + dAuxPose_Rz + ",";

		cmd = cmd + strCmdID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_WayPoint2_nj
	 * @date 2022年6月10日
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
	public int HRIF_WayPoint2_nj_base(int rbtID, int nmoveType,
								   double dEndPose_X, double dEndPose_Y, double dEndPose_Z, double dEndPose_Rx, double dEndPose_Ry, double dEndPose_Rz,
								   double dAuxPose_X, double dAuxPose_Y, double dAuxPose_Z, double dAuxPose_Rx, double dAuxPose_Ry, double dAuxPose_Rz,
								   JointsDatas joints,
								   String sTcpName, String sUcsName,
								   double dvelocity, double dacceleration, double dradius, int nisUseJoint, int nisSeek, int nIOBit, int nIOState, String strCmdID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "WayPoint2,";
		cmd = cmd + rbtID + "," + dEndPose_X + "," + dEndPose_Y + "," + dEndPose_Z + ","
				+ dEndPose_Rx + "," + dEndPose_Ry + "," + dEndPose_Rz + ",";

		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  joints.joint[i] + ",";
		}

		cmd = cmd + sTcpName + "," + sUcsName + "," + dvelocity + ","
				+ dacceleration + "," + dradius + ",";

		cmd = cmd + nmoveType + "," + nisUseJoint + "," + nisSeek + ","
				+ nIOBit + "," + nIOState + ",";

		cmd = cmd + dAuxPose_X + "," + dAuxPose_Y + "," + dAuxPose_Z + ","
				+ dAuxPose_Rx + "," + dAuxPose_Ry + "," + dAuxPose_Rz + ",";

		cmd = cmd + strCmdID + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_MoveJ
	 * @date 2022年6月10日
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
	public int HRIF_MoveJ_base(int rbtID,
			double dX, double dY, double dZ, double dRx, double dRy, double dRz,
			double dJ1, double dJ2,double dJ3,double dJ4,double dJ5,double dJ6,
			String sTcpName, String sUcsName,
			double dvelocity, double dacc, double dradius, int nIsUseJoint, int nIsSeek, int nIOBit, int nIOState, String strCmdID) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "WayPoint,";
		cmd = cmd + rbtID + "," + dX + "," + dY + "," + dZ + ","
				+ dRx + "," + dRy + "," + dRz + ",";

		cmd = cmd + dJ1 + "," + dJ2 + "," + dJ3 + ","
				+ dJ4 + "," + dJ5 + "," + dJ6 + ",";

		cmd = cmd + sTcpName + "," + sUcsName + "," + dvelocity + ","
				+ dacc + "," + dradius + ",";

		cmd = cmd + 0 + "," + nIsUseJoint + "," + nIsSeek + ","
				+ nIOBit + "," + nIOState + "," + strCmdID + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_MoveJ_nj
	 * @date 2024年10月12日
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
	public int HRIF_MoveJ_nj_base(int rbtID,
							   double dX, double dY, double dZ, double dRx, double dRy, double dRz,
							   JointsData joints,
							   String sTcpName, String sUcsName,
							   double dvelocity, double dacc, double dradius, int nIsUseJoint, int nIsSeek, int nIOBit, int nIOState, String strCmdID) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "WayPoint,";
		cmd = cmd + rbtID + "," + dX + "," + dY + "," + dZ + ","
				+ dRx + "," + dRy + "," + dRz + ",";

		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  joints.joint[i] + ",";
		}

		cmd = cmd + sTcpName + "," + sUcsName + "," + dvelocity + ","
				+ dacc + "," + dradius + ",";

		cmd = cmd + 0 + "," + nIsUseJoint + "," + nIsSeek + ","
				+ nIOBit + "," + nIOState + "," + strCmdID + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_MoveL
	 * @date 2022年6月10日
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
	public int HRIF_MoveL_base(int rbtID,
			double dX, double dY, double dZ, double dRx, double dRy, double dRz,
			double dJ1, double dJ2,double dJ3,double dJ4,double dJ5,double dJ6,
			String sTcpName, String sUcsName,
			double dvelocity, double dacc, double dradius, int nIsSeek, int nIOBit, int nIOState, String strCmdID) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "WayPoint,";
		cmd = cmd + rbtID + "," + dX + "," + dY + "," + dZ + ","
				+ dRx + "," + dRy + "," + dRz + ",";

		cmd = cmd + dJ1 + "," + dJ2 + "," + dJ3 + ","
				+ dJ4 + "," + dJ5 + "," + dJ6 + ",";

		cmd = cmd + sTcpName + "," + sUcsName + "," + dvelocity + ","
				+ dacc + "," + dradius + ",";

		cmd = cmd + 1 + "," + 0 + "," + nIsSeek + ","
				+ nIOBit + "," + nIOState + "," + strCmdID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_MoveL_nj
	 * @date 2022年6月10日
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
	public int HRIF_MoveL_nj_base(int rbtID,
							   double dX, double dY, double dZ, double dRx, double dRy, double dRz,
							   JointsDatas joints,
							   String sTcpName, String sUcsName,
							   double dvelocity, double dacc, double dradius, int nIsSeek, int nIOBit, int nIOState, String strCmdID) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "WayPoint,";
		cmd = cmd + rbtID + "," + dX + "," + dY + "," + dZ + ","
				+ dRx + "," + dRy + "," + dRz + ",";

		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  joints.joint[i] + ",";
		}

		cmd = cmd + sTcpName + "," + sUcsName + "," + dvelocity + ","
				+ dacc + "," + dradius + ",";

		cmd = cmd + 1 + "," + 0 + "," + nIsSeek + ","
				+ nIOBit + "," + nIOState + "," + strCmdID + ",;";
		return  SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_MoveC
	 * @date 2022年6月2日
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
	public int HRIF_MoveC_base(int rbtID,
			double dStartPose_X, double dStartPose_Y,double dStartPose_Z,double dStartPose_Rx,double dStartPose_Ry,double dStartPose_Rz,
			double dAuxPose_X, double dAuxPose_Y, double dAuxPose_Z, double dAuxPose_Rx, double dAuxPose_Ry, double dAuxPose_Rz,
			double dEndPose_X, double dEndPose_Y, double dEndPose_Z, double dEndPose_Rx, double dEndPose_Ry, double dEndPose_Rz,
			int nfixedPosure,
			int nMoveCType, double dRadLen, double dVelocity, double dAcceleration, double dRadius, String sTCPName, String sUCSName, String strCmdID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MoveC,";
		cmd = cmd + rbtID + "," + dStartPose_X + "," + dStartPose_Y + ","
				+ dStartPose_Z + "," + dStartPose_Rx + ","
				+ dStartPose_Ry + "," + dStartPose_Rz + ",";

		cmd = cmd + dAuxPose_X + "," + dAuxPose_Y + ","
				+ dAuxPose_Z + "," + dAuxPose_Rx + ","
				+ dAuxPose_Ry + "," + dAuxPose_Rz + ",";

		cmd = cmd + dEndPose_X + "," + dEndPose_Y + ","
				+ dEndPose_Z + "," + dEndPose_Rx + ","
				+ dEndPose_Ry + "," + dEndPose_Rz + ",";

		cmd = cmd + nfixedPosure + "," + nMoveCType + "," + dRadLen + ","
				+ dVelocity + "," + dAcceleration + "," + dRadius + ",";

		cmd = cmd + sTCPName + "," + sUCSName + "," + strCmdID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_MoveC_nj
	 * @date 2022年6月2日
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
	public int HRIF_MoveC_nj_base(int rbtID,
							   double dStartPose_X, double dStartPose_Y,double dStartPose_Z,double dStartPose_Rx,double dStartPose_Ry,double dStartPose_Rz,
							   double dAuxPose_X, double dAuxPose_Y, double dAuxPose_Z, double dAuxPose_Rx, double dAuxPose_Ry, double dAuxPose_Rz,
							   double dEndPose_X, double dEndPose_Y, double dEndPose_Z, double dEndPose_Rx, double dEndPose_Ry, double dEndPose_Rz,
							   int nfixedPosure,
							   int nMoveCType, double dRadLen, double dVelocity, double dAcceleration, double dRadius, String sTCPName, String sUCSName, String strCmdID,JointsDatas EndPointRefACS, int nIsUseCurRefACS)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MoveC,";
		cmd = cmd + rbtID + "," + dStartPose_X + "," + dStartPose_Y + ","
				+ dStartPose_Z + "," + dStartPose_Rx + ","
				+ dStartPose_Ry + "," + dStartPose_Rz + ",";

		cmd = cmd + dAuxPose_X + "," + dAuxPose_Y + ","
				+ dAuxPose_Z + "," + dAuxPose_Rx + ","
				+ dAuxPose_Ry + "," + dAuxPose_Rz + ",";

		cmd = cmd + dEndPose_X + "," + dEndPose_Y + ","
				+ dEndPose_Z + "," + dEndPose_Rx + ","
				+ dEndPose_Ry + "," + dEndPose_Rz + ",";

		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  EndPointRefACS.joint[i] + ",";
		}
		cmd = cmd + nIsUseCurRefACS + "," + nfixedPosure + "," + nMoveCType + "," + dRadLen + ","
				+ dVelocity + "," + dAcceleration + "," + dRadius + ",";

		cmd = cmd + sTCPName + "," + sUCSName + "," + strCmdID + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_MoveZ
	 * @date 2022年6月10日
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
	public int HRIF_MoveZ_base(int rbtID,
			double dStartPose_X, double dStartPose_Y, double dStartPose_Z, double dStartPose_Rx, double dStartPose_Ry, double dStartPose_Rz,
			double dEndPose_X, double dEndPose_Y, double dEndPose_Z, double dEndPose_Rx, double dEndPose_Ry, double dEndPose_Rz,
			double dPlanePos_X, double dPlanePos_Y, double dPlanePos_Z, double dPlanePos_Rx, double dPlanePos_Ry, double dPlanePos_Rz,
			double dVelocity,
			double dacc, double dwidth, double ddensity, int nEnableDensity, int nEnablePlane, int nEnableWaitTime,
			int nPosiTime, int nNegaTime, double dradius, String sTcpName, String sUcsName, String strCmdID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MoveZ,";
		cmd = cmd + rbtID + "," + dStartPose_X + "," + dStartPose_Y + ","
				+ dStartPose_Z + "," + dStartPose_Rx + ","
				+ dStartPose_Ry + "," + dStartPose_Rz + ",";

		cmd = cmd + dEndPose_X + "," + dEndPose_Y + ","
				+ dEndPose_Z + "," + dEndPose_Rx + ","
				+ dEndPose_Ry + "," + dEndPose_Rz + ",";

		cmd = cmd + dPlanePos_X + "," + dPlanePos_Y + ","
				+ dPlanePos_Z + "," + dPlanePos_Rx + ","
				+ dPlanePos_Ry + "," + dPlanePos_Rz + ",";

		cmd = cmd + dVelocity + "," + dacc + "," + dwidth + ","
				+ ddensity + "," + nEnableDensity + "," + nEnablePlane + ","
				+ nEnableWaitTime + ",";

		cmd = cmd + nPosiTime + "," + nNegaTime + "," + dradius + ","
				+ sTcpName + "," + sUcsName + "," + strCmdID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_MoveE
	 * @date 2022年6月10日
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
	public int HRIF_MoveE_base(int rbtID,
			double dP1_X, double dP1_Y, double dP1_Z, double dP1_Rx, double dP1_Ry, double dP1_Rz,
			double dP2_X, double dP2_Y, double dP2_Z, double dP2_Rx, double dP2_Ry, double dP2_Rz,
			double dP3_X, double dP3_Y, double dP3_Z, double dP3_Rx, double dP3_Ry, double dP3_Rz,
			double dP4_X, double dP4_Y, double dP4_Z, double dP4_Rx, double dP4_Ry, double dP4_Rz,
			double dP5_X, double dP5_Y, double dP5_Z, double dP5_Rx, double dP5_Ry, double dP5_Rz,
			int nMoveType, int nOrientMode, double dArcLength,
			double dvelocity, double dacc, double dradius, String sTcpName, String sUcsName, String strCmdID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MoveE,";
		cmd = cmd + rbtID + "," + dP1_X + "," + dP1_Y + "," + dP1_Z + ","
				+ dP1_Rx + "," + dP1_Ry + "," + dP1_Rz + ",";

		cmd = cmd + dP2_X + "," + dP2_Y + "," + dP2_Z + ","
				+ dP2_Rx + "," + dP2_Ry + "," + dP2_Rz + ",";

		cmd = cmd + dP3_X + "," + dP3_Y + "," + dP3_Z + ","
				+ dP3_Rx + "," + dP3_Ry + "," + dP3_Rz + ",";

		cmd = cmd + dP4_X + "," + dP4_Y + "," + dP4_Z + ","
				+ dP4_Rx + "," + dP4_Ry + "," + dP4_Rz + ",";

		cmd = cmd + dP5_X + "," + dP5_Y + "," + dP5_Z + ","
				+ dP5_Rx + "," + dP5_Ry + "," + dP5_Rz + ",";

		cmd = cmd + nMoveType + "," + nOrientMode + "," + dArcLength + ",";
		cmd = cmd + dvelocity + "," + dacc + "," + dradius + ",";
		cmd = cmd + sTcpName + "," + sUcsName + "," + strCmdID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_MoveS
	 * @date 2022年6月10日
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
	public int HRIF_MoveS_base(int rbtID, double dSpiralIncrement, double dSpiralDiameter, double dvelocity,
			double dacc, double dradius, String sTcpName, String sUcsName, String strCmdID)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MoveS" + "," + rbtID + ",";
		cmd = cmd + dSpiralIncrement + "," + dSpiralDiameter + "," + dvelocity + ",";
		cmd = cmd + dacc + "," + dradius + "," + sTcpName + "," + sUcsName + ","+ strCmdID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_MoveAlignToZ
	 * @date 2024年9月29日
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
	public int HRIF_MoveAlignToZ_base( int rbtID, String sTcpName, String sUcsName,BooleanData bIsReached,DoubleData dbJ1,DoubleData dbJ2,DoubleData dbJ3,DoubleData dbJ4,DoubleData dbJ5,DoubleData dbJ6)
	{
		int nRet =0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MoveAlignToZ" + "," + rbtID + ",";
		cmd = cmd + sTcpName + "," + sUcsName + ",;";
		nRet = SendAndReceive(cmd, rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size()!=7)
		{
			return ErrorCode.returnError.value();
		}
		bIsReached.setReturn(Integer.parseInt(rece.get(0)) == 1);
		dbJ1.setReturn(Double.parseDouble(rece.get(1)));
		dbJ2.setReturn(Double.parseDouble(rece.get(2)));
		dbJ3.setReturn(Double.parseDouble(rece.get(3)));
		dbJ4.setReturn(Double.parseDouble(rece.get(4)));
		dbJ5.setReturn(Double.parseDouble(rece.get(5)));
		dbJ6.setReturn(Double.parseDouble(rece.get(6)));
		return nRet;
	}

	/**
	 * @Title HRIF_MoveLinearWeave
	 * @date 2024年9月9日
	 * @param rbtID             机器人ID
	 * @param startPoint 开始点位置
	 * @param endPoint 结束点位置
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
	public int HRIF_MoveLinearWeave_base(int rbtID, String startPoint, String endPoint, double dvelocity, double dAcc,
										 double dRadius, double dAmplitude, double dInterValDistance, int nWeaveFrameType, double dElevation,
										 double dAzimuth, double dCentreRise, int nEnableWaitTime, double nPosiTime, double nNegaTime, String sTcpName, String sUcsName, String strCmdID,StringData vecLastPoint ) {
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MoveLinearWeave,"
				+ rbtID + ","
				+ startPoint + ","
				+ endPoint + ","
				+ dvelocity + ","
				+ dAcc + ","
				+ dRadius + ","
				+ dAmplitude + ","
				+ dInterValDistance + ","
				+ nWeaveFrameType + ","
				+ dElevation + ","
				+ dAzimuth + ","
				+ dCentreRise + ","
				+ nEnableWaitTime + ","
				+ nPosiTime + ","
				+ nNegaTime + ","
				+ sTcpName + ","
				+ sUcsName + ","
				+ strCmdID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.isEmpty())
		{
			return ErrorCode.returnError.value();
		}
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < rece.size(); i++) {
			sb.append(rece.get(i));
			if (i < rece.size() - 1) sb.append(",");
		}
		vecLastPoint.setReturn(sb.toString());
		return nRet;
	}

	/**
	 * @Title HRIF_MoveCircularWeave
	 * @date 2024年9月9日
	 * @param rbtID             机器人ID
	 * @param startPoint 开始点位置
	 * @param AuxPoint 经过点位置
	 * @param endPoint 结束点位置
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
	public int HRIF_MoveCircularWeave_base(int rbtID, String startPoint,String AuxPoint, String endPoint, double dvelocity, double dAcc,
										 double dRadius,int nOrientMode,int nMoveWhole,int nMoveWholeLen, double dAmplitude, double dInterValDistance, int nWeaveFrameType, double dElevation,
										 double dAzimuth, double dCentreRise, int nEnableWaitTime, double nPosiTime, double nNegaTime, String sTcpName, String sUcsName, String strCmdID,StringData vecLastPoint) {
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MoveCircularWeave,"
				+ rbtID + ","
				+ startPoint + ","
				+ AuxPoint + ","
				+ endPoint + ","
				+ dvelocity + ","
				+ dAcc + ","
				+ dRadius + ","
				+ nOrientMode + ","
				+ nMoveWhole + ","
				+ nMoveWholeLen + ","
				+ dAmplitude + ","
				+ dInterValDistance + ","
				+ nWeaveFrameType + ","
				+ dElevation + ","
				+ dAzimuth + ","
				+ dCentreRise + ","
				+ nEnableWaitTime + ","
				+ nPosiTime + ","
				+ nNegaTime + ","
				+ sTcpName + ","
				+ sUcsName + ","
				+ strCmdID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.isEmpty())
		{
			return ErrorCode.returnError.value();
		}
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < rece.size(); i++) {
			sb.append(rece.get(i));
			if (i < rece.size() - 1) sb.append(",");
		}
		vecLastPoint.setReturn(sb.toString());
		return nRet;
	}

	/**
 	 * @Title HRIF_MoveRelJ
	 * @param rbtID 机器人ID
	 * @param nAxisID 运动的目标轴ID 对应J1-J6
	 * @param nDirection 运动方向 0:负方向/1:正方向
	 * @param dDistance 相对运动距离
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 关节相对运动
	 */
	public int HRIF_MoveRelJ_base(int rbtID, int nAxisID, int nDirection, double dDistance) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MoveRelJ,";
		cmd = cmd + rbtID + "," + nAxisID + "," + nDirection + "," + dDistance + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_MoveRelL
	 * @param rbtID 机器人ID
	 * @param nAxisID 运动的目标轴 ID，对应空间坐标 X-Rz
	 * @param nDirection 运动方向 0:负方向 1:正方向
	 * @param dDistance 相对运动距离
	 * @param nToolMotion 运动坐标类型 0：按当前选择的用户坐标运动/1：按 Tool 坐标运动
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 空间相对运动
	 */
	public int HRIF_MoveRelL_base(int rbtID, int nAxisID, int nDirection, double dDistance, int nToolMotion) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MoveRelL,";
		cmd = cmd + rbtID + "," + nAxisID + "," + nDirection + ",";
		cmd = cmd + dDistance + "," + nToolMotion + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_WayPointRel
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
	public int HRIF_WayPointRel_base(int rbtID, int nType, int nPointList, double dPos_X, double dPos_Y, double dPos_Z, double dPos_Rx, double dPos_Ry, double dPos_Rz,
			double dPos_J1,double dPos_J2, double dPos_J3, double dPos_J4,double dPos_J5, double dPos_J6, int nrelMoveType,
			int nAxisMask_1,int nAxisMask_2,int nAxisMask_3,int nAxisMask_4,int nAxisMask_5,int nAxisMask_6,
			double dTarget1,double dTarget2,double dTarget3,double dTarget4,double dTarget5,double dTarget6,
			String sTcpName, String sUCSName, double dVelocity, double dAcc, double dRadius, int nIsUseJoint,
			int nIsSeek, int nIOBit, int nIOState, String strCmdID) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "WayPointRel,";
		cmd = cmd + rbtID + "," + nType + "," + nPointList + ",";
		cmd = cmd + dPos_X + "," + dPos_Y +"," +  dPos_Z +"," + dPos_Rx + "," + dPos_Ry +"," + dPos_Rz + ",";
		cmd = cmd + dPos_J1 + "," + dPos_J2 +"," +  dPos_J3 +"," + dPos_J4 + "," + dPos_J5 +"," + dPos_J6 + "," + nrelMoveType+ ",";
		cmd = cmd + nAxisMask_1 + "," + nAxisMask_2 +"," +  nAxisMask_3 +"," + nAxisMask_4 + "," + nAxisMask_5 +"," + nAxisMask_6 + ",";
		cmd = cmd + dTarget1 + "," + dTarget2 +"," +  dTarget3 +"," + dTarget4 + "," + dTarget5 +"," + dTarget6 + ",";
		cmd = cmd + sTcpName + "," + sUCSName +"," +  dVelocity +"," + dAcc + "," + dRadius +"," + nIsUseJoint + ",";
		cmd = cmd + nIsSeek + "," + nIOBit +"," +  nIOState +"," + strCmdID  + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_WayPointRel_nj
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
	public int HRIF_WayPointRel_nj_base(int rbtID, int nType, int nPointList, double dPos_X, double dPos_Y, double dPos_Z, double dPos_Rx, double dPos_Ry, double dPos_Rz,
										JointsDatas Pos, int nrelMoveType, JointsDatas Axis,
									 double dTarget1,double dTarget2,double dTarget3,double dTarget4,double dTarget5,double dTarget6,
									 String sTcpName, String sUCSName, double dVelocity, double dAcc, double dRadius, int nIsUseJoint,
									 int nIsSeek, int nIOBit, int nIOState, String strCmdID) {

		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "WayPointRel,";
		cmd = cmd + rbtID + "," + nType + "," + nPointList + ",";
		cmd = cmd + dPos_X + "," + dPos_Y +"," +  dPos_Z +"," + dPos_Rx + "," + dPos_Ry +"," + dPos_Rz + ",";

		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  Pos.joint[i] + ",";
		}
		cmd = cmd +  nrelMoveType+ ",";
		for (int i = 0; i < g_nRobotJoints; i++) {
			cmd +=  Axis.joint[i] + ",";
		}
		cmd = cmd + dTarget1 + "," + dTarget2 +"," +  dTarget3 +"," + dTarget4 + "," + dTarget5 +"," + dTarget6 + ",";
		cmd = cmd + sTcpName + "," + sUCSName +"," +  dVelocity +"," + dAcc + "," + dRadius +"," + nIsUseJoint + ",";
		cmd = cmd + nIsSeek + "," + nIOBit +"," +  nIOState +"," + strCmdID  + ",;";
		return SendAndReceive( cmd,rece);
	}

	/** Part11 连续轨迹运动类控制指令 */

	/**
	 * @Title HRIF_InitPath_base
	 * @date 2024年8月30日
	 * @param rbtID 机器人ID
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
	public int HRIF_InitPath_base(int rbtID, int nRawDataType,String trajectName, double dSpeedRatio, double dRadius,double dVelocity,double dAcc,double dJerk,String sUcsName,String sTcpName)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "InitPath,";
		cmd = cmd + rbtID + "," + nRawDataType + "," +trajectName + "," + dSpeedRatio + "," + dRadius + "," + dVelocity + "," + dAcc + "," + dJerk + "," + sUcsName + "," + sTcpName + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_PushPathPoints
	 * @date 2024年8月30日
	 * @param rbtID 机器人ID
	 * @param sPoints 点位数据
	 * @param trajectName 轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 向轨迹中批量推送原始点位
	 */
	public int HRIF_PushPathPoints_base(int rbtID, String trajectName, String sPoints)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "PushPathPoints,";
		cmd = cmd + rbtID + "," + trajectName + "," + sPoints + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_EndPushPathPoints
	 * @date 2024年8月30日
	 * @param rbtID 机器人ID
	 * @param sPathName 轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 结束向轨迹中推送点位
	 */
	public int HRIF_EndPushPathPoints_base(int rbtID, String sPathName)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "EndPushPathPoints,";
		cmd = cmd + rbtID + "," + sPathName + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @param rbtID          机器人ID
	 * @param dVel
	 * @param dAcc
	 * @param dTol
	 * @param rawACSpoints
	 * @param nIsSetIO
	 * @param nEndDOMask
	 * @param nEndDOVal
	 * @param nBoxDOMask
	 * @param nBoxDOVal
	 * @param nBoxCOMask
	 * @param nBoxCOVal
	 * @param nBoxAOCH0_Mask
	 * @param nBoxAOCH0_Mode
	 * @param nBoxAOCH1_Mask
	 * @param nBoxAOCH1_Mode
	 * @param dbBoxAOCH0_Val
	 * @param dbBoxAOCH1_Val
	 * @return int, 0：调用成功;>0返回错误码
	 * @Title HRIF_MovePathJOL
	 * @date 2024年9月11日
	 * @Description 启动在线实施规划的MovePathJ
	 */
	public int HRIF_MovePathJOL_base(int rbtID, double dVel, double dAcc, double dTol, String rawACSpoints,
									 String nIsSetIO, String nEndDOMask, String nEndDOVal, String nBoxDOMask, String nBoxDOVal,
									 String nBoxCOMask, String nBoxCOVal, String nBoxAOCH0_Mask, String nBoxAOCH0_Mode,
									 String nBoxAOCH1_Mask, String nBoxAOCH1_Mode, String dbBoxAOCH0_Val, String dbBoxAOCH1_Val)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		StringBuilder cmd = new StringBuilder("MovePathJOL,");
		cmd.append(rbtID).append(",")
				.append(dVel).append(",")
				.append(dAcc).append(",")
				.append(dTol).append(",");

		String[] rawPoints = rawACSpoints.split(",");
		String[] isSetIO = nIsSetIO.split(",");
		String[] endDOMask = nEndDOMask.split(",");
		String[] endDOVal = nEndDOVal.split(",");
		String[] boxDOMask = nBoxDOMask.split(",");
		String[] boxDOVal = nBoxDOVal.split(",");
		String[] boxCOMask = nBoxCOMask.split(",");
		String[] boxCOVal = nBoxCOVal.split(",");
		String[] boxAOCH0Mask = nBoxAOCH0_Mask.split(",");
		String[] boxAOCH0Mode = nBoxAOCH0_Mode.split(",");
		String[] boxAOCH1Mask = nBoxAOCH1_Mask.split(",");
		String[] boxAOCH1Mode = nBoxAOCH1_Mode.split(",");
		String[] dbBoxAOCH0Val = dbBoxAOCH0_Val.split(",");
		String[] dbBoxAOCH1Val = dbBoxAOCH1_Val.split(",");

		for (int i = 0; i < isSetIO.length; i++) {
			for (int j = 0; j < 6; j++) {
				if (6 * i + j < rawPoints.length) {
					cmd.append(rawPoints[6 * i + j]).append(",");
				}
			}
			cmd.append(isSetIO[i]).append(",")
					.append(endDOMask[i]).append(",")
					.append(endDOVal[i]).append(",")
					.append(boxDOMask[i]).append(",")
					.append(boxDOVal[i]).append(",")
					.append(boxCOMask[i]).append(",")
					.append(boxCOVal[i]).append(",")
					.append(boxAOCH0Mask[i]).append(",")
					.append(boxAOCH0Mode[i]).append(",")
					.append(boxAOCH1Mask[i]).append(",")
					.append(boxAOCH1Mode[i]).append(",")
					.append(dbBoxAOCH0Val[i]).append(",")
					.append(dbBoxAOCH1Val[i]).append(",");
		}

		if (cmd.length() > 0) {
			cmd.setLength(cmd.length() - 1); // 去掉最后一个逗号
		}
		cmd.append(";");
		return SendAndReceive(cmd.toString(),rece);
	}

	/**
	 * @Title HRIF_DelPath_base
	 * @date 2024年8月30日
	 * @param rbtID 机器人ID
	 * @param sPathName 轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 删除指定轨迹
	 */
	public int HRIF_DelPath_base(int rbtID, String sPathName)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "DelPath,";
		cmd = cmd + rbtID + "," + sPathName + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_ReadPathList
	 * @date 2024年9月11日
	 * @param rbtID 机器人ID
	 * @param result 轨迹信息
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 读取轨迹列表
	 */

	public int HRIF_ReadPathList_base(int rbtID ,StringData result)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "ReadPath,";
		cmd = cmd + rbtID  + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < rece.size(); i++) {
			sb.append(rece.get(i));
			if (i < rece.size() - 1) sb.append(",");
		}
		result.setReturn(sb.toString());
		return nRet;
	}

	/**
	 * @Title HRIF_ReadPathInfo
	 * @date 2024年8月31日
	 * @param rbtID 机器人ID
	 * @param sPathName 轨迹名称
	 * @param result 轨迹信息
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 读取轨迹信息
	 */

	public int HRIF_ReadPathInfo_base(int rbtID, String sPathName,StringData result)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "ReadPath,";
		cmd = cmd + rbtID + "," + sPathName + ",;";
		nRet = SendAndReceive(cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() != 32)
		{
			return ErrorCode.returnError.value();
		}
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < rece.size(); i++) {
			sb.append(rece.get(i));
			if (i < rece.size() - 1) sb.append(","); // Add separator if needed
		}
		result.setReturn(sb.toString()); // Assuming setValue method exists in StringData
		return nRet;
	}

	/**
	 * @Title HRIF_UpdatePathName
	 * @date 2024年8月31日
	 * @param rbtID 机器人ID
	 * @param sPathName 原轨迹名称
	 * @param sPathNewName 新轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 更新指定轨迹的名称
	 */
	public int HRIF_UpdatePathName_base(int rbtID, String sPathName, String sPathNewName)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "UpdatePathName,";
		cmd = cmd + rbtID + "," + sPathName + "," + sPathNewName + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_ReadPathState
	 * @date 2024年8月31日
	 * @param rbtID 机器人ID
	 * @param sPathName 原轨迹名称
	 * @param nStateJ 轨迹的MovePathJ状态
	 * @param nErrorCodeJ MovePathJ状态对应的错误码
	 * @param nStateL 轨迹的MovePathL状态
	 * @param nErrorCodeL MovePathL状态对应的错误码
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 读取当前轨迹状态
	 */
	public int HRIF_ReadPathState_base(int rbtID, String sPathName, IntData nStateJ, IntData nErrorCodeJ,
									   IntData nStateL, IntData nErrorCodeL)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "ReadPathState,";
		cmd = cmd + rbtID + "," + sPathName + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() != 4)
		{
			return ErrorCode.returnError.value();
		}
		nStateJ.setReturn(Integer.parseInt(rece.get(0)));
		nErrorCodeJ.setReturn(Integer.parseInt(rece.get(1)));
		nStateL.setReturn(Integer.parseInt(rece.get(2)));
		nErrorCodeL.setReturn(Integer.parseInt(rece.get(3)));
		return nRet;
	}

	/**
	 * @Title HRIF_StartPushMovePathJ
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @param dSpeedRatio 轨迹运动速度比
	 * @param dRadius 过渡半径
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 初始化关节连续轨迹运动
	 */
	public int HRIF_StartPushMovePathJ_base(int rbtID, String sTrackName, double dSpeedRatio, double dRadius)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "StartPushMovePath,";
		cmd = cmd + rbtID + "," + sTrackName + "," + dSpeedRatio + "," + dRadius + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_PushMovePathJ
	 * @date 2022年6月2日
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
	public int HRIF_PushMovePathJ_base(int rbtID, String sTrackName, double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6) {
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "PushMovePathJ,";
		cmd = cmd + rbtID + "," + sTrackName + "," +  dJ1 + "," + dJ2
				+ "," +  dJ3 + "," +  dJ4 + ","
				+  dJ5 + "," +  dJ6 + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_EndPushMovePathJ
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 轨迹下发完成并开始计算轨迹
	 */
	public int HRIF_EndPushMovePathJ_base(int rbtID, String sTrackName)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "EndPushMovePath,";
		cmd = cmd + rbtID + "," + sTrackName + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_MovePathJ
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 运动指定的轨迹
	 */
	public int HRIF_MovePathJ_base(int rbtID, String sTrackName) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MovePath,";
		cmd = cmd + rbtID + "," + sTrackName + ",;";
		return SendAndReceive(cmd,rece);
	}


	/**
	 * @Title HRIF_ReadMovePathState
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @param nState 轨迹状态
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前的轨迹状态
	 */
	public int HRIF_ReadMovePathJState_base(int rbtID, String sTrackName, IntData nState) {
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadMovePathState,";
		cmd = cmd + rbtID + "," + sTrackName + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() != 1)
		{
			return ErrorCode.returnError.value();
		}
		nState.setReturn(Integer.parseInt(rece.get(0)));
		return nRet;
	}


	/**
	 * @Title HRIF_UpdateMovePathJName
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹原名称
	 * @param sTrackNewName 更新的轨迹名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 更新指定轨迹的名称
	 */
	public int HRIF_UpdateMovePathJName_base(int rbtID, String sTrackName, String sTrackNewName)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "UpdateMovePathName,";
		cmd = cmd + rbtID + "," + sTrackName + "," + sTrackNewName +",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_DelMovePath
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹原名称
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 删除指定轨迹
	 */
	public int HRIF_DelMovePathJ_base(int rbtID, String sTrackName)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "DelMovePath,";
		cmd = cmd + rbtID + "," + sTrackName +",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_GetMovePathJOLIndex
	 * @date 2024年9月11日
	 * @param rbtID 机器人ID
	 * @param getIndexForPoint 点位索引号
	 * @param getTotalPoints 总点数
	 * @return int, 0：调用成功;>0返回错误码
	 * @Description 获取MovePathJOL运动当前的点位索引号及轨迹运动所有点总数。
	 */
	@SuppressWarnings("unchecked")
	public int HRIF_GetMovePathJOLIndex_base(int rbtID, IntData getIndexForPoint, IntData getTotalPoints)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "GetMovePathJOLIndex,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive(cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() != 2)
		{
			return ErrorCode.returnError.value();
		}
		getIndexForPoint.setReturn(Integer.parseInt(rece.get(0)));
		getTotalPoints.setReturn(Integer.parseInt(rece.get(1)));
		return nRet;
	}

	/**
	 * @Title HRIF_ReadTrackProcess
	 * @date 2022年6月2日
	 * @param rbtID:机器人ID
	 * @param dProcess 轨迹运行进度
	 * @param nIndex 点位索引
	 * @return int, 0 调用成功;>0 返回错误码
	 * @Description 读取当前的轨迹运动进度
	 */
	@SuppressWarnings("unchecked")
	public int HRIF_ReadTrackProcess_base(int rbtID, DoubleData dProcess, IntData nIndex)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadSoftMotionProgress,";
		cmd = cmd + rbtID + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() != 2)
		{
			return ErrorCode.returnError.value();
		}
		dProcess.setReturn(Double.parseDouble(rece.get(0)));
		nIndex.setReturn(Integer.parseInt(rece.get(1)));
		return nRet;
	}

	/**
	 * @Title HRIF_InitMovePathL
	 * @date 2022年6月10日
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
	public int HRIF_InitMovePathL_base(int rbtID, String sTrackName, double dvelocity, double dacceleration, double djerk,
			String sUcsName, String sTcpName) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "InitMovePathL,";
		cmd = cmd + rbtID + "," + sTrackName + "," + dvelocity + "," + dacceleration + "," + djerk + "," + sUcsName
				+ "," + sTcpName + ",;";
		return SendAndReceive(cmd,rece);
	}


	/**
	 * @Title HRIF_PushMovePathL
	 * @date 2022年6月2日
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
	public int HRIF_PushMovePathL_base(int rbtID, String sTrackName, double dx, double dy, double dz, double drx, double dry,
			double drz) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "PushMovePathL,";
		cmd = cmd + rbtID + "," + sTrackName + "," +  dx + "," +  dy
				+ "," +  dz + "," + drx + "," +  dry
				+ "," +  drz + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_PushMovePaths
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @param nMoveType 点位类型
	 * @param nPointsSize 点位数量
	 * @param sPoints 点位数据
	 * @return int, 0:调用成功;>0返回错误码
	 * @Description 批量下发轨迹点位，调用一次可下发多个点位数据
	 */
	public int HRIF_PushMovePaths_base(int rbtID, String sTrackName, int nMoveType, int nPointsSize, String sPoints)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		List<String> MovePoints = Arrays.asList(sPoints.split(","));
		if(MovePoints.size() != nPointsSize*6) {
			nRet = (ErrorCode.paramsError.value());
			return nRet;
		}
		String cmd = "PushMovePaths," + rbtID + ","+ sTrackName + "," + nMoveType + "," + nPointsSize + ",";
		for(int i = 0; i<MovePoints.size();i++){
			cmd = cmd + MovePoints.get(i) + ",";
		}
		cmd = cmd + ";";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_MovePathL
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param sTrackName 轨迹名称
	 * @return int, 0:调用成功;>0返回错误码
	 * @Description 执行空间坐标轨迹运动
	 * @Attention 调用HRIF_MovePathL后会计算完轨迹后直接开始运动，计算时间2-4s左右，根据实际轨迹大小确定。
	 */
	public int HRIF_MovePathL_base(int rbtID, String sTrackName)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "MovePathL,";
		cmd = cmd + rbtID + "," + sTrackName + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @param rbtID 机器人ID
	 * @param dOverride 速度比
	 * @return int, 0:调用成功;>0返回错误码
	 * @Description 设置 MovePath 运动速度比
	 */
	public int HRIF_SetMovePathOverride_base (int rbtID, double dOverride) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetMovePathOverride,";
		cmd = cmd + rbtID + "," + dOverride + ",;";
		return SendAndReceive( cmd,rece);
	}

	/** Part12 Servo运动类控制指令 */

	/**
	 * @Title HRIF_StartServo
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param dservoTime 更新周期
	 * @param dlookaheadTime 前瞻时间
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 启动机器人在线控制（ servoJ 或 servoP）时，设定位置固定更新的周期和前瞻时间
	 */
	public int HRIF_StartServo_base(int rbtID, double dservoTime, double dlookaheadTime) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "StartServo,";
		cmd = cmd + rbtID + "," + dservoTime + "," + dlookaheadTime
				+ ",;";
		return SendAndReceive(cmd,rece);
	}


	/**
	 * @Title HRIF_PushServoJ
	 * @date 2022年6月2日
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
	public int HRIF_PushServoJ_base(int rbtID, double dJ1,double dJ2,double dJ3,double dJ4,double dJ5,double dJ6) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "PushServoJ,";
		cmd = cmd + rbtID + "," + dJ1 + "," + dJ2 + "," + dJ3 + ","
				+ dJ4 + "," + dJ5 + "," + dJ6 + ",;";
		return SendAndReceive( cmd,rece);
	}


	/**
	 * @Title HRIF_PushServoP
	 * @date 2022年6月2日
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
	public int HRIF_PushServoP_base(int rbtID, double dX, double dY, double dZ,double dRx,double dRy,double dRz,
			double dTCP_X, double dTCP_Y, double dTCP_Z, double dTCP_Rx, double dTCP_Ry, double dTCP_Rz,
			double dUCS_X, double dUCS_Y, double dUCS_Z, double dUCS_Rx, double dUCS_Ry, double dUCS_Rz) {
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "PushServoP,";
		cmd = cmd + rbtID + "," + dX + "," + dY + "," + dZ + ","
				+ dRx + "," + dRy + "," + dRz + ","
				+ dUCS_X + "," + dUCS_Y + "," + dUCS_Z + ","
				+ dUCS_Rx + "," + dUCS_Ry + "," + dUCS_Rz + ","
				+ dTCP_X + "," + dTCP_Y + "," + dTCP_Z + ","
				+ dTCP_Rx + "," + dTCP_Ry + "," + dTCP_Rz + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SpeedJ
	 * @param rbtID     机器人ID
	 * @param dJ1CmdVel 关节速度
	 * @param dJ2CmdVel
	 * @param dJ3CmdVel
	 * @param dJ4CmdVel
	 * @param dJ5CmdVel
	 * @param dJ6CmdVel
	 * @param dAcc      加速度
	 * @param dRunTime  运行时长
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2024年9月7日
	 * @Description 在线关节运动速度伺服控制，以该指令中指定的各个关节的速度和加速度运动指定的时长。
	 */
	public int HRIF_SpeedJ_base(int rbtID, double dJ1CmdVel, double dJ2CmdVel, double dJ3CmdVel, double dJ4CmdVel, double dJ5CmdVel, double dJ6CmdVel, double dAcc, double dRunTime)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "SpeedJ," + rbtID + "," + dJ1CmdVel + "," + dJ2CmdVel + ","
				+ dJ3CmdVel + "," + dJ4CmdVel + "," + dJ5CmdVel + ","
				+ dJ6CmdVel + "," + dAcc + "," + dRunTime + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SpeedL
	 * @param rbtID       机器人ID
	 * @param dXCmdVel X/Y/Z速度
	 * @param dYCmdVel
	 * @param dZCmdVel
	 * @param dRxCmdVel Rx/Ry/Rz 速度
	 * @param dRyCmdVel
	 * @param dRzCmdVel
	 * @param dLinearAcc X/Y/Z加速度
	 * @param dAngularAcc Rx/Ry/Rz 加速度
	 * @param dRunTime    运行时长
	 * @return int, 0:调用成功;>0:返回错误码
	 * @date 2024年9月7日
	 * @Description 3.12.5.1.在线空间运动速度伺服控制，以该指令中指定的位姿各个坐标的速度和加速度运动指定的时长。
	 */
	public int HRIF_SpeedL_base(int rbtID, double dXCmdVel, double dYCmdVel, double dZCmdVel, double dRxCmdVel, double dRyCmdVel, double dRzCmdVel, double dLinearAcc, double dAngularAcc, double dRunTime)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "SpeedL," + rbtID + "," + dXCmdVel + "," + dYCmdVel + ","
				+ dZCmdVel + "," + dRxCmdVel + "," + dRyCmdVel + ","
				+ dRzCmdVel + "," + dLinearAcc + "," + dAngularAcc + ","
				+ dRunTime + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_InitServoEsJ
	 * @date 2022年6月14日
	 * @param rbtID 机器人ID
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 初始化在线控制模式，清空缓存点位 ,ServoEsJ
	 */
	public int HRIF_InitServoEsJ_base(int rbtID)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "InitServoEsJ" + "," + rbtID + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_StartServoEsJ
	 * @date 2022年6月14日
	 * @param rbtID 机器人ID
	 * @param dServoTime 更新周期
	 * @param dLookaheadTime 前瞻时间
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 启动在线控制模式，设定位置固定更新的周期和前瞻时间，开始运动
	 */
	public int HRIF_StartServoEsJ_base(int rbtID, double dServoTime, double dLookaheadTime)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "StartServoEsJ" + "," + rbtID + ",";
		cmd = cmd + dServoTime + "," + dLookaheadTime + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_PushServoEsJ
	 * @date 2022年6月14日
	 * @param rbtID 机器人ID
	 * @param nPointSize 点位数量
	 * @param sPoints 点位信息
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 批量下发在线控制点位 ,每个点位下发频率由固定更新的周期确定
	 */
	public int HRIF_PushServoEsJ_base(int rbtID, int nPointSize, String sPoints)
	{
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		List<String> MovePoints = Arrays.asList(sPoints.split(","));
		if(nPointSize*7 != MovePoints.size()) {
			return ErrorCode.paramsError.value();
		}
		String cmd = "PushServoEsJ" + "," + rbtID + "," + nPointSize + ",";
		for(int i = 0; i<MovePoints.size();i++){
			cmd = cmd + MovePoints.get(i) + ",";
		}
		cmd = cmd + ";";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_ReadServoEsJState
	 * @date 2022年6月14日
	 * @param rbtID 机器人ID
	 * @param Ret 点位状态
	 * @param nIndex 当前点位索引
	 * @param nCount 点位数量
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 读取当前是否可以继续下发点位信息，循环读取间隔 >20ms
	 */
	public int HRIF_ReadServoEsJState_base(int rbtID, IntData Ret, IntData nIndex, IntData nCount)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadServoEsJState" + "," + rbtID +",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet!=ErrorCode.REC_Succeed.value())
		{
			return nRet;
		}
		if(rece.size() != 3)
		{
			return ErrorCode.returnError.value();
		}
		Ret.setReturn(Integer.parseInt(rece.get(0)));
		nIndex.setReturn(Integer.parseInt(rece.get(1)));
		nCount.setReturn(Integer.parseInt(rece.get(2)));
		return nRet;
	}

	/** Part13 相对跟踪运动类控制指令 */

	/**
	 * @Title HRIF_SetMoveTraceParams
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param nState 跟踪状态
	 * @param dDistance 相对跟踪运动保持的相对距离
	 * @param dAwayVelocity 相对跟踪的运动的探寻速度
	 * @param dBackVelocity 往返速度
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置相对跟踪运动控制参数
	 */
	public int HRIF_SetMoveTraceParams_base(int rbtID, int nState, double dDistance, double dAwayVelocity, double dBackVelocity)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetMoveTraceParams,";
		cmd = cmd + rbtID +"," + nState + "," + dDistance + "," + dAwayVelocity + "," + dBackVelocity + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetMoveTraceInitParams
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param dK 传感器计算参数
	 * @param dB 传感器计算参数
	 * @param dMaxLimit 激光传感器检测距离最大值
	 * @param dMinLimit 激光传感器检测距离最小值
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置相对跟踪运动初始化参数
	 */
	public int HRIF_SetMoveTraceInitParams_base(int rbtID, double dK, double dB, double dMaxLimit, double dMinLimit)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetMoveTraceInitParams,";
		cmd = cmd + rbtID +"," + dK + "," + dB + "," + dMaxLimit +"," + dMinLimit+ ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_SetMoveTraceUcs
	 * @date 2022年6月2日
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
	public int HRIF_SetMoveTraceUcs_base(int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double dRz)
	{
		Vector<String> rece = new Vector<String>();
		rbtID = 0;
		String cmd = "SetMoveTraceUcs,";
		cmd = cmd + rbtID +"," + dX + "," + dY + "," + dZ +"," + dRx+ "," + dRy + "," + dRz + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetTrackingState
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param nState 跟踪状态
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置传送带跟踪运动状态
	 */
	public int HRIF_SetTrackingState_base(int rbtID, int nState)
	{
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetTrackingState,";
		cmd = cmd + rbtID +"," + nState + ",;";
		return SendAndReceive(cmd,rece);
	}

	/** Part14 位置跟随运动类指令 **/

	/**
	 * @Title HRIF_SetPoseTrackingMaxMotionLimit_base
	 * @date 2023年2月27日
	 * @param rbtID 机器人ID
	 * @param dMaxLineVel 直线最大速度
	 * @param dMaxOriVel 姿态最大速度
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置位置跟随的最大跟随速度
	 */
	public int HRIF_SetPoseTrackingMaxMotionLimit_base( int rbtID, double dMaxLineVel, double dMaxOriVel) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetPoseTrackingMaxMotionLimit,";
		cmd = cmd + rbtID +"," + dMaxLineVel + ","+ dMaxOriVel+ ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetPoseTrackingStopTimeOut
	 * @date 2024年9月11日
	 * @param rbtID 机器人ID
	 * @param dTime 超时时间
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置位置跟踪超时停止时间
	 */
	public int HRIF_SetPoseTrackingStopTimeOut_base( int rbtID, double dTime ) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetPoseTrackingMaxMotionLimit,";
		cmd = cmd + rbtID +"," + dTime + ",;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_SetPoseTrackingPIDParams_base
	 * @date 2023年2月27日
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
	public int HRIF_SetPoseTrackingPIDParams_base(int rbtID, double dPosPID1, double dPosPID2, double dPosPID3, double dOriPID1, double dOriPID2, double dOriPID3) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetPoseTrackingPIDParams,";
		cmd = cmd + rbtID +"," + dPosPID1 + ","+ dPosPID2+ "," + dPosPID3 + ",";
		cmd = cmd + dOriPID1 + "," + dOriPID2 + "," + dOriPID1 +",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetPoseTrackingTargetPos_base
	 * @date 2023年2月27日
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
	public int HRIF_SetPoseTrackingTargetPos_base( int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double dRz) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetPoseTrackingTargetPos,";
		cmd = cmd + rbtID +"," + dX + ","+ dY+ "," + dZ + ",";
		cmd = cmd + dRx + "," + dRy + "," + dRz +",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetPoseTrackingState_base
	 * @date 2023年2月27日
	 * @param rbtID 机器人ID
	 * @param nState 位置跟随状态
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 设置位置跟随状态
	 */
	public int HRIF_SetPoseTrackingState_base(int rbtID, int nState) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetPoseTrackingState,";
		cmd = cmd + rbtID +"," + nState + ",;";
		return SendAndReceive( cmd,rece);
	}

	/**
	 * @Title HRIF_SetUpdateTrackingPose_base
	 * @date 2023年2月27日
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
	public int HRIF_SetUpdateTrackingPose_base(int rbtID, double dX, double dY, double dZ, double dRx, double dRy, double dRz) {
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "SetUpdateTrackingPose,";
		cmd = cmd + rbtID +"," + dX + ","+ dY+ "," + dZ + ",";
		cmd = cmd + dRx + "," + dRy + "," + dRz +",;";
		return SendAndReceive(cmd,rece);
	}
	/** Part15 其他指令 */

	/**
	 * @Title HRIF_HRAppCmd
	 * @date 2022年6月2日
	 * @param HRAppName App名称
	 * @param HRAppCmdName 命令名称
	 * @param param 参数列表
	 * @return int, 0:调用成功;>0:返回错误码
	 * @Description 执行插件 app命令
	 */
	public int HRIF_HRAppCmd_base(String HRAppName, String HRAppCmdName, String param)
	{
		Vector<String> rece = new Vector<String>();
		List<String> AppParam = Arrays.asList(param.split(","));
		String cmd = "HRAppCmd," + HRAppName + ",";
		if(HRAppCmdName != "") 
		{
			cmd = cmd + HRAppCmdName +",";
		}
		if(AppParam.size()>0) {
			for (String s : AppParam) {
				if(s == "") {
					s = " ";
				}
				cmd = cmd + s + ",";
			}
			cmd = cmd + ";";
		}else {
			cmd = cmd + " ,;";
		}
		return SendAndReceive( cmd,rece);
	}

	public int HRIF_IsHRAppCmdExist_base()
	{
		Vector<String> rece = new Vector<String>();
		String cmd = "IsHRAppCmdExist,;";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_WriteEndHoldingRegi
	 * @date 2022年6月2日
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
	public int HRIF_WriteEndHoldingRegisters_base(int rbtID, int nSlaveID, int nFunction, int nRegAddr, int nRegCount, Vector<Integer> VecData)
	{
		rbtID=0;
		Vector<String> rece = new Vector<String>();
		if (nRegCount != VecData.size()) {
			return ErrorCode.paramsError.value();
		}
		String cmd = "WriteHoldingRegisters,";
		cmd = cmd + rbtID + "," + nSlaveID + "," + nFunction + ",";
		cmd = cmd + nRegAddr + "," + nRegCount +",";
		for(int i = 0 ;i<nRegCount;i++) {
			cmd = cmd + VecData.get(i) + ",";
		}
		cmd = cmd + ";";
		return SendAndReceive(cmd,rece);
	}

	/**
	 * @Title HRIF_ReadEndHoldingRegisters
	 * @date 2022年6月2日
	 * @param rbtID 机器人ID
	 * @param nSlaveID 从站ID
	 * @param nFunction 功能码
	 * @param nRegAddr 寄存器地址
	 * @param nRegCount 寄存器数量
	 * @return int, 0:调用成功;>0:返回错误码
 	 * @Description 读末端连接的 modbus从站寄存器，
	 * @Attention 末端为EtherCAT总线版本IO时有效
	 */
	@SuppressWarnings("unchecked")
	public int HRIF_ReadEndHoldingRegisters_base(int rbtID, int nSlaveID, int nFunction, int nRegAddr, int nRegCount, Vector<Integer> Ret)
	{
		int nRet = 0;
		rbtID = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "ReadHoldingRegisters,";
		cmd = cmd + rbtID + "," + nSlaveID + "," + nFunction + ",";
		cmd = cmd + nRegAddr + "," + nRegCount + ",;";
		nRet = SendAndReceive( cmd,rece);
		if(nRet != 0)
		{
			return nRet;
		}
		if( rece.size() != nRegCount) {
			return ErrorCode.returnError.value();
		}
		Ret.clear();
		for(int i=0;i<nRegCount;i++) {
			Ret.add(Integer.parseInt(rece.get(i)));
		}
		return nRet;
	}

	public boolean HRIF_UpLoadFile_base(String FilePath, String FileName)
	{
		String BoxIP = m_ip;
		if(FileName == null || StringUtils.isBlank(FileName))
		{
			System.out.println("No File");
			return false;
		}
		//文件名判断，不可与一下文件重名
		String [] filename = {"HRBase.py", "ScriptMain.py","UserScript.py","CCClient.py","ScriptDefine.py","Socket.py","UserInfo.py"};
		for(int i = 0; i < filename.length; i++)
		{
			if(filename[i].equals(FileName))
			{
				System.out.println("File already exists");
				return false;
			}

		}
		FilePath = FilePath + FileName;
		try {
			boolean bNet = connect("/UserPythonScript", BoxIP, 21, "ftp", "hrftp");
			if(!bNet)
			{
				return false;
			}
			File file = new File(FilePath);
			upload(file);
			//Thread.sleep(1000);
			int nRet = HRIF_CopyUserPythonScript_Base(FileName);
			if(0 != nRet)
			{
				System.out.println("HRIF_CopyFile false, Error code :"+nRet);
				return false;
			}
		} catch (Exception e) {
			// TODO 自动生成的 catch 块
			e.printStackTrace();
			return false;
		}
		return true;
	}

	/**
	 *
	 * @param path 上传到ftp服务器哪个路径下
	 * @param addr 地址
	 * @param port 端口号
	 * @param username 用户名
	 * @param password 密码
	 * @return
	 * @throws Exception
	 */
	private  boolean connect(String path,String addr,int port,String username,String password) throws Exception {
		boolean result = false;
		ftp = new FTPClient();
		int reply;
		ftp.connect(addr,port);
		ftp.login(username,password);
		ftp.enterLocalPassiveMode();
		ftp.setFileType(FTPClient.BINARY_FILE_TYPE);
		reply = ftp.getReplyCode();
		if (!FTPReply.isPositiveCompletion(reply)) {
			ftp.disconnect();
			return result;
		}
		ftp.changeWorkingDirectory(path);
		result = true;
		return result;
	}

	/**
	 *
	 * @param file 上传的文件或文件夹
	 * @throws Exception
	 */
	private void upload(File file) throws Exception{
		if(file.isDirectory())
		{
			ftp.makeDirectory(file.getName());
			ftp.changeWorkingDirectory(file.getName());
			String[] files = file.list();
			for (int i = 0; i < files.length; i++)
			{
				File file1 = new File(file.getPath()+"\\"+files[i] );
				if(file1.isDirectory())
				{
					upload(file1);
					ftp.changeToParentDirectory();
				}else{
					File file2 = new File(file.getPath()+"\\"+files[i]);
					FileInputStream input = new FileInputStream(file2);
					boolean bNet = ftp.storeFile(file2.getName(), input);
					if(bNet)
					{
						System.out.println("ftp.storeFile false");
						return;
					}
					input.close();
				}
			}
		}else
		{
			File file2 = new File(file.getPath());
			FileInputStream input = new FileInputStream(file2);
			ftp.storeFile(file2.getName(), input);
			input.close();
		}
	}

	/**
	 * @Title HRIF_CopyUserPythonScript_Base
	 * @date 2023年3月8日
	 * @param FileName 文件名
	 */
	public int HRIF_CopyUserPythonScript_Base(String FileName)
	{
		//文件名判断，不可与以下文件重名
		if(FileName == null || StringUtils.isBlank(FileName))
		{
			return ErrorCode.paramsError.value();
		}
		String [] filename = {"HRBase.py", "ScriptMain.py","UserScript.py","CCClient.py","ScriptDefine.py","Socket.py","UserInfo.py"};
		for(int i = 0; i < filename.length; i++)
		{
			if(filename[i].equals(FileName))
			{
				return ErrorCode.paramsError.value();
			}
		}
		int nRet = 0;
		Vector<String> rece = new Vector<String>();
		String cmd = "CopyUserPythonScript,";
		cmd = cmd + FileName + ",;";
		nRet = SendAndReceive(cmd,rece);
		if(nRet == 0)
		{
			if(rece.get(1).equals(OK) && rece.size() == 3) {
				nRet = 0;
			}else if(rece.get(1).equals(Fail)) {
				nRet = Integer.parseInt(rece.get(2));
			}else {
				nRet = ErrorCode.returnError.value();
			}
		}
		return nRet;
	}

	long  GetLastCallTime()
	{
		long lastCallTIme=m_IFSocket.GetLastCallTime();
		if(lastCallTIme<m_IFSocket_FastPort .GetLastCallTime())
		{
			lastCallTIme=m_IFSocket_FastPort.GetLastCallTime();
		}
		return lastCallTIme;
	}

	public static void main(String[] args) {
		// int a = Connect("172.18.34.165",10003);
		// double b = ReadOverride(3);
	}
}