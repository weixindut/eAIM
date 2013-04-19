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
import java.util.List;
import java.util.Set;

/**
 *
 * @author Administrator
 */
public class DeleteNode extends NS3Commond{
    private List<Integer> IDSet;
    public DeleteNode(List<Integer> idset)
    {
        super(4+1+4*idset.size(),DELETENODES);
        this.IDSet=idset;
    }
    public List<Integer> getIDSet()
    {
        return this.IDSet;
    }
    public boolean sendtoNS3(Socket socket)
            throws IOException
    {
        OutputStream ops=socket.getOutputStream();
        DataOutputStream dos= new DataOutputStream(ops);
        dos.writeInt(this.getCommondLength());
        dos.writeByte(this.getCommondID());
        for(int nodeID: this.IDSet)
        {
            dos.writeInt(nodeID);
        }
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
