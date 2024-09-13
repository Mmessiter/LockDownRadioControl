// ** GPS.h **

#include <Arduino.h>
#include "Hardware/1Definitions.h"
#ifdef USE_LOCAL_GPS
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
    float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(lat1) * cos(lat2) *
            sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float distance = EARTH_RADIUS * c;
    return distance * 1000; // convert to meters
}

// *************************************************************

void GetGPSTime()
{
    Hours_TX_GPS    =   GPS.hour;
    Mins_TX_GPS     =   GPS.minute;
    Secs_TX_GPS     =   GPS.seconds;
    Day_TX_GPS      =   GPS.day;
    Month_TX_GPS    =   GPS.month;
    Year_TX_GPS     =   GPS.year;
}

// *************************************************************

void CalculateGPSAverages(){
    Local_GPS_Readings_Count++;
    LatitudeGPS_Sum         += Latitude_TX_GPS;
    LongitudeGPS_Sum        += Longitude_TX_GPS;
    AltitudeGPS_Sum         += Attitude_TX_GPS;
    LatitudeGPS_Average     =  LatitudeGPS_Sum  / (float) Local_GPS_Readings_Count;
    LongitudeGPS_Average    =  LongitudeGPS_Sum / (float) Local_GPS_Readings_Count;
    AltitudeGPS_Average     =  AltitudeGPS_Sum  / (float) Local_GPS_Readings_Count;
}
// *************************************************************
void CalculateGPSCorrections(){
    LatitudeGPS_Correction  = LatitudeGPS_Average  - Latitude_TX_GPS;
    LongitudeGPS_Correction = LongitudeGPS_Average - Longitude_TX_GPS;
    AltitudeGPS_Correction  = AltitudeGPS_Average  - Attitude_TX_GPS;
}
// *************************************************************

void GetGPSLocation()
{
    Latitude_TX_GPS     = GPS.latitudeDegrees;
    Longitude_TX_GPS    = GPS.longitudeDegrees;
    Attitude_TX_GPS     = GPS.altitude;
  //  SpeedGPS        = GPS.speed;
  //  AngleGPS        = GPS.angle;
  //  Satellites_TX_GPS   = GPS.satellites;
    CalculateGPSAverages();
    CalculateGPSCorrections();
    Distance_TX_GPS     = GetDistance(GPS_RX_Latitude + LatitudeGPS_Correction, GPS_RX_Longitude + LongitudeGPS_Correction, StoredLatitude_TX_GPS, StoredLongitude_TX_GPS);
    Course_TX_ToGPS     = GetHeading(GPS_RX_Latitude + LatitudeGPS_Correction, GPS_RX_Longitude + LongitudeGPS_Correction, StoredLatitude_TX_GPS, StoredLongitude_TX_GPS);
  
}
// *************************************************************

void ReadGPS()
{
    static uint32_t timer = millis();
    GPS.read();
    if (GPS.newNMEAreceived()) if (!GPS.parse(GPS.lastNMEA())) return; // receive and parse ... this also sets the newNMEAreceived() flag to false
    if (millis() - timer > 1000)
    {
        timer = millis();
        GOT_A_FIX_TX_GPS = GPS.fix;
        if (GOT_A_FIX_TX_GPS)
        {
            GetGPSTime(); // maybe later
            GetGPSLocation();
            CalculateGPSAverages();
            CalculateGPSCorrections();
        }
    }
}

  #endif
#endif
