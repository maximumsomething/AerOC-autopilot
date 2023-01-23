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

#include <math.h>
#include <eigen.h>
//#include <Eigen/Geometry>
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
	using Eigen::Vector2f;

	NMEAGPS gps;
	gps_fix fix;
	NeoGPS::Location_t currentLoc;
	NeoGPS::Location_t targetLoc;

	float trueBearing; //calibrated bearing from inertial + time-averaged error relative to true north/gps heading. TODO - implement code that maintains this number.
	float bearingError; //a number by which we rotate our reference rotation about the unit z axis by to make it point towards true north.
	float bearingToTarget; //calculated bearing to our target location

	Vector2f velocity = Vector2f::Zero(); // x is north, y is west

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
		}
	}

    //Vector2f posErrorAvg = Vector2f::Zero(); //store the exponentially weighted time average of position errors. x is distance y is bearing
    //float posSmoothing = .2;

	void updatenav() {
        
		if(gps.available( gpsPort )) {
			fix = gps.read();
			updateClock();

            //maintain an exponentially weighted time average of position errors which we add to each new fix. Error vector goes from GPS fix to inertial location
            //Vector2f curPosError = Vector2f::Zero(); //x north y is west
            //curPosError[0] = fix.location.DistanceKm(curLocation) * cosf(fix.location.BearingToDegrees(curLocation)) * 1000.0 * posSmoothing;
            //curPosError[1] = fix.location.DistanceKm(curLocation) * -sinf(fix.location.BearingToDegrees(curLocation)) * 1000.0 * posSmoothing;;           
            
			currentLoc = fix.location;

			bearingToTarget = fix.location.BearingToDegrees(targetLoc);
			float speed_mps = fix.speed_metersph() / 3600.0;
			float heading_rad = fix.heading() / 180.0 * M_PI;
			velocity = Vector2f(cos(heading_rad) * speed_mps, -sin(heading_rad) * speed_mps);
        }else{
            velocity[1] += DeadReckoner::getGPSVelocity()[0];
            velocity[0] += DeadReckoner::getGPSVelocity()[1];
                
            offsetBearing = atan2(velocity[1], velocity[0]);
            offsetDistance = (sqrt(powf(velocity[1], 2)+ powf(velocity[0],2)) * .02); //Offset distance for currentLocation this tick in m
            curLocation = curLocation.OffsetBy(offsetBearing, offsetDistance);
		}

        bearingToTarget = curLocation.BearingToDegrees(tarLocation);

		trace_all( DEBUG_PORT, gps, fix );
	}
}
