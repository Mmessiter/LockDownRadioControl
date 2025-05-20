
// ** GPS.h **
#ifndef GPS_H
#define GPS_H
#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Wire);



// *************************************************************

void setupGPS()
{
    GPS.begin(0x10);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    delay(1000);
    GPS.println(PMTK_Q_RELEASE); // ??
}
// *************************************************************

float GetHeading(float lat1, float lon1, float lat2, float lon2)
{
    // Convert latitude and longitude from degrees to radians
    lat1 = radians(lat1);
    lon1 = radians(lon1);
    lat2 = radians(lat2);
    lon2 = radians(lon2);
    float dLon = lon2 - lon1;
    float x = sin(dLon) * cos(lat2);
    float y = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(dLon));
    float initial_bearing = atan2(x, y);
    initial_bearing = degrees(initial_bearing);
    float compass_bearing = fmod((initial_bearing + 360), 360);
    return compass_bearing;
}

// *************************************************************

float GetDistance(float lat1, float lon1, float lat2, float lon2)
{
    const float EARTH_RADIUS = 6371.0; // Earth's radius in kilometers
    lat1 = radians(lat1);
    lon1 = radians(lon1);
    lat2 = radians(lat2);
    lon2 = radians(lon2);
    float dLat = lat2 - lat1;
    float dLon = lon2 - lon1;
    float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float distance = EARTH_RADIUS * c;
    return distance * 1000; // convert to meters
}

// *************************************************************

void GetGPSTime()
{
    HoursGPS = GPS.hour;
    MinsGPS = GPS.minute;
    SecsGPS = GPS.seconds;
    DayGPS = GPS.day;
    MonthGPS = GPS.month;
    YearGPS = GPS.year;
}
// ********************************
double MetresToYards(double metres)
{
    return metres * 1.09361;
}
// ************************************************************
float KnotsToMph(float knots)
{
    return knots * 1.15078; // Convert knots to mph
}
//**************************************************

void GetGPSLocation()
{
    LatitudeGPS = GPS.latitudeDegrees;
    LongitudeGPS = GPS.longitudeDegrees;
    AltitudeGPS = MetersToFeet(GPS.altitude);
    SpeedGPS = KnotsToMph(GPS.speed);
    AngleGPS = GPS.angle;
    SatellitesGPS = GPS.satellites;
    DistanceGPS = MetresToYards(GetDistance(LatitudeGPS, LongitudeGPS, StoredLatitudeGPS, StoredLongitudeGPS));
    CourseToGPS = GetHeading(LatitudeGPS, LongitudeGPS, StoredLatitudeGPS, StoredLongitudeGPS);
}

// *************************************************************
void MarkHere()
{
    StoredLatitudeGPS = LatitudeGPS;
    StoredLongitudeGPS = LongitudeGPS;
}

// *************************************************************

void ReadGPS()
{
    if (!GPS_Connected) // If GPS is not connected, do not read
        return;

    static uint32_t timer = 0;
    if (millis() - timer < 998)
        return; // read about once per sec
    timer = millis();
    GPS.read();
    if (GPS.newNMEAreceived())
    {
        if (!GPS.parse(GPS.lastNMEA()))
            return; // receive and parse ... this also sets the newNMEAreceived() flag to false
    }
    GpsFix = GPS.fix;
    if (GpsFix)
    {
        GetGPSTime();
        GetGPSLocation();
    }
}

#endif
