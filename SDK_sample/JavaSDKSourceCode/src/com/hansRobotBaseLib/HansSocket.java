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

public class HansSocket
{
    Socket m_socket ;
    DataOutputStream m_send ;
    DataInputStream m_receive ;
    boolean m_bConnect = false;
    boolean m_bAutoTestState =false;
    long m_lastCallTime = System.currentTimeMillis();

    int initSocket(String hostName, int port)
    {
        try
        {
            m_socket = new Socket(hostName, port);
            m_send = new DataOutputStream(m_socket.getOutputStream());
            m_receive = new DataInputStream(m_socket.getInputStream());
            m_bConnect = true;
            return 0;
        }
        catch (IOException e)
        {
            if(port == 10003)
            {
                e.printStackTrace();
            }
            m_bConnect = false;
            return ErrorCode.Connect2CPSFailed.value();
        }
    }
    public boolean isConnected()
    {
        synchronized (this)
        {
            if (!m_bConnect)
            {
                if (m_socket != null && !m_socket.isClosed())
                {
                    try {
                        m_socket.close();  // Windows 下关闭 socket
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
                m_socket = null;  // 等效于 m_hSocket = -1;
                return false;
            }
            return true;
        }
    }

    public void disconnect()
    {
        synchronized (this) {
            if (m_socket != null)
            {
                try {
                    if (!m_socket.isClosed())
                    {
                        m_socket.close();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
                m_socket = null;
                m_bConnect = false;
            }
        }
    }

    public void SetAutoTestState(boolean state)
    {
        m_bAutoTestState=state;
    }

    public int RecvMsg(String cmd,Vector<String> receive)
    {
        int res=0;
        long startTime;
        byte[] buffer = new byte[1024];
        StringBuilder strRecv = new StringBuilder();
        startTime = System.currentTimeMillis();
        if (m_receive != null)
        {
            do {
                try {
                    int available = m_receive.available();
                    int toRead = Math.min(available, buffer.length);
                    int len = (toRead > 0) ? m_receive.read(buffer, 0, toRead) : -1;

                    if (len > 0) {
                        strRecv.append(new String(buffer, 0, len));
                        // 判断是否读完（例如以 ; 结尾）
                        if (strRecv.length() > 0 && strRecv.charAt(strRecv.length() - 1) == ';') {
                            break;
                        }
                    } else {
                        Thread.sleep(10);  // 避免忙等
                    }
                } catch (IOException | InterruptedException e) {
                    return ErrorCode.SocketError.value();
                }
                if ((System.currentTimeMillis() - startTime) > 10000) {
                    return ErrorCode.SocketError.value();
                }

            } while (true);
        }
        else
        {
            if (m_bConnect)
            {
                res = ErrorCode.SocketError.value();
            }
            else
            {
                res = ErrorCode.isNotConnect.value();
            }
        }
        return HanleReceiveMsg(strRecv,receive,cmd);
    }

    public int HanleReceiveMsg(StringBuilder receiveMsg,Vector<String> rece,String cmd)
    {
        int res=0;
        if (m_bAutoTestState)
        {
            System.out.println(receiveMsg);
        }
        if (receiveMsg.indexOf(",") >= 0 && receiveMsg.charAt(receiveMsg.length() - 1) == ';')
        {
            String cleaned = receiveMsg.substring(0, receiveMsg.length() - 1);
            String[] strs = cleaned.split(",");
            for (String str : strs)
            {
                rece.add(str.trim());
            }
        }
        else
        {
            return ErrorCode.returnError.value();
        }
        if(rece == null ||rece.size() < 2)
        {
            return ErrorCode.returnError.value();
        }
        // 提取命令名（strCmd 的第一个字段）
        String[] cmdParts = cmd.split(",");
        String expectedCmdName = cmdParts[0].trim();  // 第一个字段作为命令名

        // 对比返回的第一个字段
        String actualCmdName = rece.get(0);
        if (!actualCmdName.equals(expectedCmdName)) {
            return ErrorCode.CmdError.value();
        }
        rece.remove(0);  // 删除命令名
        // 提取结果字段
        String strResult = rece.get(0);
        rece.remove(0);  // 删除结果字段
        UpdateLastCallTime();

        if (strResult.equals("OK"))
        {
            return 0;
        } else if (strResult.equals("Fail"))
        {
            if (!rece.isEmpty())
            {
                try
                {
                    return Integer.parseInt(rece.get(0));
                } catch (NumberFormatException e) {
                    return ErrorCode.paramsError.value();
                }
            }
            return ErrorCode.paramsError.value();
        }
        return ErrorCode.paramsError.value();
    }



    int doSendAndReceive(String cmd,Vector<String> vecReceive)
    {
        int res = 0;
        if (m_bAutoTestState)
        {
            System.out.println(cmd);
        }

        try {
            if (m_send != null && m_bConnect) {
                //Socket发送数据
                m_send.write(cmd.getBytes());
            } else {
                m_bConnect = false;
                return ErrorCode.isNotConnect.value();
            }
            return RecvMsg(cmd,vecReceive);
        }
        catch (Exception e)
        {
            m_bConnect = false;
            e.printStackTrace();
            try
            {
                m_send.flush();
                m_send.close();
                m_receive.close();
                m_socket.close();
            }
            catch (IOException e1)
            {
                e1.printStackTrace();
            }
            return ErrorCode.isNotConnect.value();
        }
    }


    public void UpdateLastCallTime()
    {
        synchronized (this) {
            m_lastCallTime = System.currentTimeMillis();
        }
    }

    public long GetLastCallTime()
    {
        synchronized (this)
        {
        return m_lastCallTime;
        }
    }

}



