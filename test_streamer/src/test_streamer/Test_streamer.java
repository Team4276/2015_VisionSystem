/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package test_streamer;

/**
 *
 * @author acappon
 */
public class Test_streamer
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
