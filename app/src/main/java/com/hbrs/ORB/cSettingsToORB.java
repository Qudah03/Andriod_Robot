//package com.example.orb;
package com.hbrs.ORB;

import java.nio.ByteBuffer;

public class cSettingsToORB
{
	public cSettingsToORB()
	{
	}

	public int fill( ByteBuffer buffer )
	{
		int idx = 2;
		synchronized(this)
		{
			buffer.put(idx++, (byte) 6); //id
			buffer.put(idx++, (byte) 0); // reserved

			//


			buffer.put(idx++, command);

			command = 0;

			// name
			for (int i = 0; i < 20; i++) {
			    if(i<name.length())
				    buffer.put(idx++, (byte) name.charAt( i ));
			    else
                    buffer.put(idx++, (byte)(0));
			}
            buffer.put(idx++, (byte) (0));

			buffer.put(idx++, VccOk);
			buffer.put(idx++, VccLow);

		}
      return ( idx-2 );
	}

	public boolean isNew = true;

	String name = new String("");
	byte VccOk = 0;
	byte VccLow = 0;
	byte command = 0;


    //---------------------------------------------------------------
    public void setSettings( String name, double VccOk, double VccLow)
    {
        synchronized( this )
        {
            command = 1;
            this.name = name;
            this.VccOk = (byte)(10.0*VccOk);
            this.VccLow = (byte)(10.0*VccLow);

            isNew=true;
        } // synchronized
    }


}
