def getTileID(PositionWGS84, nTileLevel):
    nX = int(getX_LLH2TC(PositionWGS84[0], nTileLevel))
    nY = int(getY_LLH2TC(PositionWGS84[1], nTileLevel))
    return __getTileID(nX, nY, nTileLevel)

def getX_LLH2TC(dLon, nTileLevel):
    dTempLon = dLon + 180.
    while (dTempLon < 0.0):
        dTempLon += 360.0
    while (dTempLon >= 360.0):
        dTempLon -= 360.0

    return dTempLon / 360.0 * pow(2, nTileLevel)

def getY_LLH2TC(dLat, nTileLevel):
    dTempLat = dLat + 90.
    while (dTempLat < 0.0):
        dTempLat += 360.0
    while (dTempLat >= 360.0):
        dTempLat -= 360.0
    
    return dTempLat / 360.0 * pow(2, nTileLevel)

def __getTileID(x, y, nTileLevel):
    EncodedMortonVal = encodeMorton2(x, y);
    tile_id = EncodedMortonVal | pow(2, nTileLevel * 2)
    return tile_id

def encodeMorton2(x, y):
    tempval = (part1By1(y) << 1) + part1By1(x)
    return tempval

def part1By1(x):
    tempX = int(x)
    tempX = (tempX ^ (tempX << 16)) & 0x0000ffff0000ffff
    tempX = (tempX ^ (tempX << 8)) & 0x00ff00ff00ff00ff
    tempX = (tempX ^ (tempX << 4)) & 0x0f0f0f0f0f0f0f0f
    tempX = (tempX ^ (tempX << 2)) & 0x3333333333333333
    tempX = (tempX ^ (tempX << 1)) & 0x5555555555555555
    return tempX