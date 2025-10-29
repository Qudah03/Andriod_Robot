//package com.example.orb;
package com.hbrs.ORB;

import java.nio.ByteBuffer;

public class cSettingsFromORB
{
	public cSettingsFromORB()
	{
		for (int i = 0; i < 20; i++) {
			name[i] = 0;
		}
		name[20] = 0;
	}
	
	public boolean get( ByteBuffer data )
	{
		int idx = 0;

		short crc =  (short)( ( data.get(idx++) & 0xFF)
				|( data.get(idx++) & 0xFF) << 8);

		byte id =  (byte)( ( data.get(idx++) & 0xFF) );
		byte reserved =  (byte)( ( data.get(idx++) & 0xFF) );

		if( id != 5 )
			return false;
		// TODO: check CRC !

		synchronized (this) {

            versionMain = (short)( ((short)data.get(idx++) & 0xFF) | ((short)data.get(idx++) & 0xFF) << 8);
            versionSub  = (short)( ((short)data.get(idx++) & 0xFF) | ((short)data.get(idx++) & 0xFF) << 8);
            boardMain   = (short)( ((short)data.get(idx++) & 0xFF) | ((short)data.get(idx++) & 0xFF) << 8);
            boardSub    = (short)( ((short)data.get(idx++) & 0xFF) | ((short)data.get(idx++) & 0xFF) << 8);

			for (int i = 0; i < 20; i++) {
				name[i] = data.get(idx++);
			}
			name[20] = 0;
			idx++;

			vcc_ok  = (byte)(data.get( idx++) & 0xFF);
            vcc_low = (byte)(data.get( idx++) & 0xFF);
		}
		return true;
	}

    public String getName()
    {
        String str = new String(name,0,20);

        return( str );
    }

    public String getHW_Version()
    {
        String str = String.format( "%02d.%02d", boardMain, boardSub );
        return( str );
    }

    public String getSW_Version()
    {
        String str = String.format( "%02d.%02d", versionMain, versionSub );
        return( str );
    }

    public double getVccOK()
    {
        return( 0.1*vcc_ok );
    }

    public double getVccLow()
    {
        return( 0.1*vcc_low );
    }

    // must be public, getter must use synchronized

	public byte name[] = new byte[21];
    public short versionMain = 0;
    public short versionSub  = 0;
    public short boardMain = 0;
    public short boardSub  = 0;

    public byte vcc_ok = 0;
    public byte vcc_low = 0;

}
