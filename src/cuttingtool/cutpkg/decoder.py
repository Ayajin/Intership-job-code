class tile_range:
    lon_low = None
    lon_high = None
    lat_low = None
    lat_high = None

def decode_by_tile_id(ullTileID, m_nTileLevel):
    nTileX = decodeMorton2X(ullTileID)
    nTileY = decodeMorton2Y(ullTileID)

    tile_range_struct = tile_range()

    dLon_Left = getLon_TC2LLH(nTileX, m_nTileLevel)
    dLon_Right = getLon_TC2LLH(nTileX + 1., m_nTileLevel)
    dLat_Bottom = getLat_TC2LLH(nTileY, m_nTileLevel)
    dLat_Top = getLat_TC2LLH(nTileY + 1., m_nTileLevel)

    tile_range_struct.lon_high = dLon_Right
    tile_range_struct.lon_low = dLon_Left
    tile_range_struct.lat_high = dLat_Top
    tile_range_struct.lat_low = dLat_Bottom

    return tile_range_struct

def decodeMorton2X(nTileID):
    return compact1By1(nTileID >> 0)

def decodeMorton2Y(nTileID):
    return compact1By1(nTileID >> 1)

def compact1By1(x):
    x &= 0x5555555555555555 
    x = (x ^ (x >> 1)) & 0x3333333333333333 
    x = (x ^ (x >> 2)) & 0x0f0f0f0f0f0f0f0f 
    x = (x ^ (x >> 4)) & 0x00ff00ff00ff00ff 
    x = (x ^ (x >> 8)) & 0x0000ffff0000ffff 
    x = (x ^ (x >> 16)) & 0x00000000ffffffff 

    return int(x)

def getLon_TC2LLH(X_TC, nTileLevel):
    dTempLon = X_TC * 360.0 / pow(2, nTileLevel)
    while (dTempLon < 0.0):
        dTempLon += 360.0
    while (dTempLon >= 360.0):
        dTempLon -= 360.0
    dTempLon -= 180.
    
    return dTempLon

def getLat_TC2LLH(Y_TC, nTileLevel):
    dTempLat = Y_TC * 360.0 / pow(2, nTileLevel)
    while (dTempLat < 0.0):
        dTempLat += 360.0
    while (dTempLat >= 360.0):
        dTempLat -= 360.0
    dTempLat -= 90.
    
    return dTempLat