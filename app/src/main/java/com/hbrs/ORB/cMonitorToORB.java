//package com.example.orb;
package com.hbrs.ORB;

import java.nio.ByteBuffer;

public class cMonitorToORB
{
	public cMonitorToORB()
	{
	}
	

	public int fill( ByteBuffer buffer )
	{
		int idx = 2;
		synchronized(this)
		{
			buffer.put(idx++, (byte) 3); //id
			buffer.put(idx++, (byte) 0); // reserved

			// mode
			buffer.put(idx++, (byte) ((mode)      & 0xFF));
			buffer.put(idx++, (byte) ((parameter) & 0xFF));
			buffer.put(idx++, (byte) ((keycode)   & 0xFF));


		}
      return ( idx-2 );
	}


	public byte                 mode = 0;
	public byte                 parameter = 0;
	public byte                 keycode = 0;
}
