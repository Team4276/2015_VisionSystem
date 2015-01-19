/*******************************************************************************************/
/* The MIT License (MIT)                                                                   */
/*                                                                                         */
/* Copyright (c) 2014 - Marina High School FIRST Robotics Team 4276 (Huntington Beach, CA) */
/*                                                                                         */
/* Permission is hereby granted, free of charge, to any person obtaining a copy            */
/* of this software and associated documentation files (the "Software"), to deal           */
/* in the Software without restriction, including without limitation the rights            */
/* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell               */
/* copies of the Software, and to permit persons to whom the Software is                   */
/* furnished to do so, subject to the following conditions:                                */
/*                                                                                         */
/* The above copyright notice and this permission notice shall be included in              */
/* all copies or substantial portions of the Software.                                     */
/*                                                                                         */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR              */
/* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,                */
/* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE             */
/* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER                  */
/* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,           */
/* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN               */
/* THE SOFTWARE.                                                                           */
/*******************************************************************************************/

/*******************************************************************************************/
/* We are a high school robotics team and always in need of financial support.             */
/* If you use this software for commercial purposes please return the favor and donate     */
/* (tax free) to "Marina High School Educational Foundation"  (Huntington Beach, CA)       */

/*******************************************************************************************/


class CConnection
{
public:

    typedef enum
    {
        CONNECTION_STATE_RESET,
        CONNECTION_STATE_INIT,
        CONNECTION_STATE_BIND_OK,
        CONNECTION_STATE_ACCEPT_OK,
    } CONNECTION_STATE;

    CConnection();
    ~CConnection();

    void init(int port, char* pName);
    std::string displayText() const;
    void showConnectionContext(const char* func, const char* msg) const;
    static std::string connectionStateToText(CONNECTION_STATE eState);

    int serverBind();
    int acceptConnection();
    void resetConnection();
    void tryToBind();
    void tryToAccept();
    int writeClient(char* pBuf, unsigned int siz);
    int readClient(char* pBuf, unsigned int siz);

    virtual void calledOnceAfterAccept(void)
    {
    }

    int m_port;
    int m_sock;
    int m_client;
    CONNECTION_STATE m_state;
    pthread_t m_threadSender;
    pthread_t m_threadListener;
    char m_name[32];
};

class CBrowserConnection : public CConnection
{
public:

    void calledOnceAfterAccept(void)
    {
        // Tell the browser that what follows is an mjpeg stream
        char head[512];
        sprintf(head, "HTTP/1.0 200 OK\r\nServer: mjpeg-streamer\r\nContent-Type: multipart/x-mixed-replace;boundary=informs\r\n--informs\r\n");
        writeClient(head, strlen(head));
        fflush(NULL);
    }
};

class CTextConnection : public CConnection
{
public:

    CLedStrip m_ledStrip;
};
