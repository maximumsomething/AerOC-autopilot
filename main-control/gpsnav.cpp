// Prevent GPSport.h from being included
#define GPSport_h
// Configure our own serial port for the parser
#define gpsPort Serial5
#define GPS_PORT_NAME "Serial5"
#define DEBUG_PORT Serial
// Include the header after the above are configured
#include <NMEAGPS.h>
// For printing GPS output
#include <Streamers.h>
#include <eigen.h>
#include <Eigen/Geometry>
//Project includes
#include "sensorcomm.h"
#include "inertial.h"
#include "pilot.h"
#include "ringbuffer.h"
// For setting the clock
#include <TimeLib.h>
#include <NeoTime.h>

namespace GPSNav {
    using Eigen::Vector3f;
	using Eigen::Quaternionf;

    NMEAGPS gps;
    gps_fix fix;
    NeoGPS::Location_t currentLoc;
    NeoGPS::Location_t targetLoc;

    float trueBearing; //calibrated bearing from inertial + time-averaged error relative to true north/gps heading. TODO - implement code that maintains this number.
    float bearingError; //a number by which we rotate our reference rotation about the unit z axis by to make it point towards true north.
    float bearingToTarget; //calculated bearing to our target location

    float nSpeed; //Northerly speed in m/s
    float eSpeed; //Easterly speed in m/s
    Vector3f GPSAccel = Vector3f::Zero();

    time_t getTeensy3Time() {
    return Teensy3Clock.get();
    }

    void gpsSetup() {
        gpsPort.begin(9600);
        // Sets TimeLib provider
        setSyncProvider(getTeensy3Time);
    }

    // Update the RTC using the current GPS fix time
    void updateClock() {
        if (fix.valid.time) {
            Serial.println("Setting RTC");
            uint32_t secondsSince2000 = fix.dateTime;
            // Convert to Jan 1, 1970 epoch
            Teensy3Clock.set(secondsSince2000 + 946684800);
            // Doesn't work???
        }
    }

    void updatenav() {
        if(gps.available( gpsPort )) {
            fix = gps.read();
            updateClock();
            currentLoc = Location_t(fix.lattitudeDMS.degrees, fix.longitudeDMS.degrees);

            bearingToTarget = fix.location.BearingToDegrees(targetLoc);
            nSpeed = fix.velocity_north/100.0;
            eSpeed = fix.velocity_east/100.0;
        }else{
            GPSAccel = DeadReckoner::getCalibratedAccel(); //this is meant to be accel s.t. x is north and z is up and whatever
            GPSAccel[2] = 0;
        }

        trace_all( DEBUG_PORT, gps, fix );
    }
}
