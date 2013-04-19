/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package aim4.ns3commond;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;

/**
 *
 * @author Administrator
 */
public class RunToTimestep extends NS3Commond{
    private int timeStep;
    public RunToTimestep(int time)
    {
        super(9,RUNTOTIMESTEP);
        this.timeStep=time;
    }
    public int getTimeStep()
    {
        return this.timeStep;
    }
    public boolean sendtoNS3(Socket socket)
            throws IOException
    {
        OutputStream ops=socket.getOutputStream();
        DataOutputStream dos= new DataOutputStream(ops);
        dos.writeInt(this.getCommondLength());
        dos.writeByte(this.getCommondID());
        dos.writeInt(timeStep);
        //下面的代码多次重复使用考虑提取出来，待优化
        InputStream is= socket.getInputStream();   
        DataInputStream dis = new DataInputStream(is);
        int length = dis.readInt();
        byte commondID = dis.readByte();
        byte status = dis.readByte();
        Result result = new Result(status);
        if( length==6 && commondID==COMMONDEXESTATUS)
        {
            System.err.printf("receive a delete nodes result from ns3, and the status is %d\n",result.getResult());
            ops.close();
            dos.close();
            is.close();
            dis.close();
            return result.isExeOK();
        }
        else
        {
            System.err.printf(" when waiting for a create node commond result, receive an unspect result\n",result.getResult());
            ops.close();
            dos.close();
            is.close();
            dis.close();
            return false;
        }                        
    }
}
