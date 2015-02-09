/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package test_streamer;

import java.io.*;
import java.net.*;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author acappon
 */
public class Main
{

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args)
    {
        JReceiver receiver = new JReceiver();
        receiver.init();
        if (receiver.m_initOK)
        {
            receiver.getFromSocket();
        } else
        {
            System.out.println("CReceiver client did not initialize propeerly -- make sure server is running");
        }
    }
}
