/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package aim4.ns3commond;

/**
 *
 * @author Administrator
 */
public class Result extends NS3Commond{
    private byte exeStatus;
    public Result(byte status)
    {
        super(6,COMMONDEXESTATUS);
        this.exeStatus=status;
    }
    public byte getResult()
    {
        return this.exeStatus;
    }
    public boolean isExeOK()
    {
        if(this.exeStatus==0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
}
