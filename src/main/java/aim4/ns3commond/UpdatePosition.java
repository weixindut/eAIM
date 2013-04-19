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
public class UpdatePosition extends NS3Commond{
    private float nodeX;
    private float nodeY;
    private int nodeID;
    public UpdatePosition( float x,float y,int id)
    {
        super(4+1+4+4+4,UPDATEPOSITON);
        this.nodeX=x;
        this.nodeY=y;
        this.nodeID=id;
    }
    public float getNodeX()
    {
        return this.nodeX;
    }
    public float getNodeY()
    {
        return this.nodeY;
    }
    public int getNodeID()
    {
        return this.nodeID;
    }
    public boolean sendtoNS3(Socket socket)
            throws IOException
    {
        OutputStream ops=socket.getOutputStream();
        DataOutputStream dos= new DataOutputStream(ops);
        dos.writeInt(this.getCommondLength());
        dos.writeByte(this.getCommondID());
        dos.writeInt(this.nodeID);
        dos.writeFloat(this.nodeX);
        dos.writeFloat(this.nodeY);
        
        InputStream is= socket.getInputStream();   
        DataInputStream dis = new DataInputStream(is);
        int length = dis.readInt();
        byte commondID = dis.readByte();
        byte status = dis.readByte();
        Result result = new Result(status);
        if( length==6 && commondID==COMMONDEXESTATUS)
        {
            System.err.printf("receive a update node position result from ns3, and the status is %d\n",result.getResult());
            ops.close();
            dos.close();
            is.close();
            dis.close();
            return result.isExeOK();
        }
        else
        {
            System.err.printf(" when waiting for a update node position commond result, receive an unspect result\n",result.getResult());
            ops.close();
            dos.close();
            is.close();
            dis.close();
            return false;
        }                        
    }
}
