//package com.example.orb;
package com.hbrs.ORB;

import java.nio.ByteBuffer;

public class cMonitorFromORB
{
	public cMonitorFromORB()
	{
		for(int i=0;i<4*31;i++)
			text[i]=0;
		line = 0;
	}
	
	public boolean get( ByteBuffer data )
	{
		int idx = 0;

		short crc =  (short)( ( data.get(idx++) & 0xFF)
				|( data.get(idx++) & 0xFF) << 8);

		byte id =  (byte)( ( data.get(idx++) & 0xFF) );
		byte reserved =  (byte)( ( data.get(idx++) & 0xFF) );

		if( id != 4 )
		    return false;
		// TODO: check CRC !

		synchronized (this) {

            line = data.get(idx++);
            if (0<=line && line < 4)
            {
                for (int i = 0; i < 30; i++) {
                    text[i + line * 31] = data.get(idx++);
                }
                text[(line + 1) * 31-1]='\n';
            }
		}
		return true;
	}


	  // must be public, getter must use synchronized
	public  byte[]  text  = new byte[4*31];
	private byte   line;
}
