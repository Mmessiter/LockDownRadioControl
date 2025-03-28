
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
void DisplayGPSTime()
{
    Serial.print("\nTime: ");
    Serial.print(HoursGPS, DEC);
    Serial.print(':');
    Serial.print(MinsGPS, DEC);
    Serial.print(':');
    Serial.println(SecsGPS, DEC);
    Serial.print("Date: ");
    Serial.print(DayGPS, DEC);
    Serial.print('/');
    Serial.print(MonthGPS, DEC);
    Serial.print("/");
    Serial.println(YearGPS, DEC);
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
    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(lat1) * cos(lat2) *
                  sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float distance = EARTH_RADIUS * c;
    return distance * 1000; // convert to meters
}

// *************************************************************

void DisplayGPSLocation()
{
    Serial.print("Latitude: ");
    Serial.println(LatitudeGPS, 4);
    Serial.print("Longitude: ");
    Serial.println(LongitudeGPS, 4);
    Serial.print("Speed (knots): ");
    Serial.println(SpeedGPS); //< Current speed over ground in knots
    Serial.print("Current heading: ");
    Serial.println(AngleGPS); //< Course in degrees from true north
    Serial.print("Altitude: ");
    Serial.println(AltitudeGPS); //< Altitude in meters above MSL
    Serial.print("Satellites: ");
    Serial.println(SatellitesGPS); //< Number of satellites in use
    Serial.print("Distance to mark: ");
    Serial.println(DistanceGPS, 5);
    Serial.print("Heading to mark: ");
    Serial.println(CourseToGPS, 5);
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
    static uint32_t timer = millis();
    GPS.read();
    if (GPS.newNMEAreceived())
        if (!GPS.parse(GPS.lastNMEA()))
            return; // receive and parse ... this also sets the newNMEAreceived() flag to false
    if (millis() - timer > 1000)
    {
        timer = millis();
        GpsFix = GPS.fix;

        if (GpsFix)
        {
            GetGPSTime();
            GetGPSLocation();
            DisplayGPSTime();
            DisplayGPSLocation();
        }
        else
        {
            Serial.print("Minutes: ");
            Serial.print(millis() / 60000);
            Serial.print(" Satellites: ");
            Serial.println((int)GPS.satellites);
        }
    }
}

#endif
