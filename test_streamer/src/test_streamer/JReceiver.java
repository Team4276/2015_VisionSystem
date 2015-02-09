/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package test_streamer;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author acappon
 */
public class JReceiver
{

    JTargetInfo m_currentTargetInfo = null;
    String m_host = "192.168.1.120";
    DatagramSocket m_bbbTextSocket = null;
    BufferedReader m_in = null;
    Boolean m_initOK = false;

    void init()
    {
        m_currentTargetInfo = new JTargetInfo();
        m_initOK = false;
        try 
        {
            m_bbbTextSocket = new DatagramSocket();
         }
        catch(SocketException e)
        {
            System.err.println("SocketException");
            m_initOK = false;
            m_in = new BufferedReader(new InputStreamReader(System.in));
       }
        if (m_in == null)
        {
            System.err.println("IN == null");
            m_initOK = false;
        }

        m_initOK = true;
    }

    void getFromSocket()
    {
        try
        {
            Integer iCount = 0;
            String result;
            InetAddress IPAddress = InetAddress.getByName("192.168.1.120");
            byte[] receiveData = new byte[1024];
            while (true)
            {
                DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length, IPAddress, 5801);
                m_bbbTextSocket.receive(receivePacket);
                String textIn = new String(receivePacket.getData());
                m_currentTargetInfo.initTargetInfoFromText(textIn);
                iCount++;
                result = iCount.toString();
                result += " -- ";
                result += textIn;
                System.out.println(result);
            }
        } catch (IOException ex)
        {
            Logger.getLogger(JReceiver.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
}
