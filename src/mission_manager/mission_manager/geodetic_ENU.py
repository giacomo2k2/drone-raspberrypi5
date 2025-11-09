import math

# WGS84
_a = 6378137.0
_f = 1.0 / 298.257223563
_e2 = _f * (2 - _f)

def geodetic_to_ecef(lat_deg, lon_deg, h):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    N = _a / math.sqrt(1 - _e2 * math.sin(lat)**2)
    x = (N + h) * math.cos(lat) * math.cos(lon)
    y = (N + h) * math.cos(lat) * math.sin(lon)
    z = (N * (1 - _e2) + h) * math.sin(lat)
    return x, y, z

def ecef_to_enu(x, y, z, lat0_deg, lon0_deg, h0):
    x0, y0, z0 = geodetic_to_ecef(lat0_deg, lon0_deg, h0)
    dx = x - x0
    dy = y - y0
    dz = z - z0

    phi = math.radians(lat0_deg)
    lam = math.radians(lon0_deg)

    sin_phi = math.sin(phi); cos_phi = math.cos(phi)
    sin_lam = math.sin(lam); cos_lam = math.cos(lam)

    # rotation matrix multiply R * d
    east  = -sin_lam * dx + cos_lam * dy
    north = -sin_phi * cos_lam * dx - sin_phi * sin_lam * dy + cos_phi * dz
    up    =  cos_phi * cos_lam * dx + cos_phi * sin_lam * dy + sin_phi * dz

    return east, north, up

# convenience: lat/lon/alt -> ENU relative to origin
def geodetic_to_enu(lat, lon, h, lat0, lon0, h0):
    x, y, z = geodetic_to_ecef(lat, lon, h)
    return ecef_to_enu(x, y, z, lat0, lon0, h0)

