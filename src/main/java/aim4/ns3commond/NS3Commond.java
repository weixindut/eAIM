/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package aim4.ns3commond;

import aim4.sim.*;

/**
 *
 * @author Administrator
 */
public abstract class NS3Commond {
    public static final byte RUNTOTIMESTEP=0;
    public static final byte CREATENODE=1;
    public static final byte DELETENODES=2;
    public static final byte UPDATEPOSITON=3;
    public static final byte SCHEDULEMESSAGES=4;
    public static final byte RECEIVERESULTS=5;
    public static final byte COMMONDEXESTATUS=6;
    
    
    private int CommondLength;
    private byte CommondID;
    
    public NS3Commond(int length, byte id)
    {
        this.CommondLength = length;
        this.CommondID = id;
    }
    public NS3Commond( NS3Commond nc)
    {
        this.CommondID=nc.CommondID;
        this.CommondLength=nc.CommondLength;
    }
    public int getCommondLength()
    {
        return this.CommondLength;
    }
    public byte getCommondID()
    {
        return this.CommondID;
    }
}
