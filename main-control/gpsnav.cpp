//Project includes
#include "gpsnav.h"
#include "sensorcomm.h"
#include "inertial.h"
#include "pilot.h"
#include "ringbuffer.h"
#include "telemetry.h"

// Instead of editing library files, we are adding definitions here to avoid synchronization nightmares
// Prevent GPSport.h from being included
#define GPSport_h
// Configure our own serial port for the parser
#define gpsPort Serial5
#define GPS_PORT_NAME "Serial5"
#define DEBUG_PORT Serial

// Change the configuration according to the output of the NMEAOrder.ino example code
#include <NMEAGPS_cfg.h>
#undef LAST_SENTENCE_IN_INTERVAL
#define LAST_SENTENCE_IN_INTERVAL NMEAGPS::NMEA_GLL
#include <NMEAGPS.h>

// For printing GPS output
#include <Streamers.h>

#include <math.h>
#include <eigen.h>
//#include <Eigen/Geometry>
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
	float target_dist; //how many meters from a target before we consider it to have been "reached"

	bool enableGPSNav = true;
	ring_buffer<NeoGPS::Location_t> waypoints(1);
	float bearingError; //a number by which we rotate our reference rotation about the unit z axis to make it point towards true north. Time-averaged difference between gps heading and inertial heading
	float bearingToTarget; //calculated bearing to our target location

	float bearingSmoothing = .5; //the smoothing coefficient for true bearing averaging

	time_t getTeensy3Time() {
		//Serial.println("getTeensy3Time");
		return Teensy3Clock.get();
	}

	//North end of athletics field: 41.300093, -82.224700. South end of athletics field: 41.298565, -82.224848
	void setWaypoints(float routePoints[][2], int numWaypoints){
		waypoints = ring_buffer<NeoGPS::Location_t>(numWaypoints);
		for(int i = 0; i < numWaypoints; i++){
			waypoints.put(NeoGPS::Location_t(routePoints[i][0], routePoints[i][1]));
		}
		targetLoc = waypoints.get(0);
	}
	
	void gpsSetup() {
		gpsPort.begin(9600);
		float waypointArray[2][2] = {{41.300093, -82.224700},{41.298565, -82.224848}};
		setWaypoints(waypointArray, 2);
	}

	// Update the RTC using the current GPS fix time
	void updateClock() {
		if (fix.valid.time) {
			uint32_t secondsSince2000 = fix.dateTime;
			//Serial.printf("Setting RTC: Seconds since epoch: %d\n", secondsSince2000);
			// Convert to Jan 1, 1970 epoch
			Teensy3Clock.set(secondsSince2000 + 946684800);
			setSyncProvider(&getTeensy3Time);
		}
	}

    //Vector2f posErrorAvg = Vector2f::Zero(); //store the exponentially weighted time average of position errors. x is distance y is bearing
    //float posSmoothing = .2;

	void updatenav() {
        
		if(gps.available( gpsPort )) {
			trace_all( DEBUG_PORT, gps, fix );
			fix = gps.read();
			updateClock();

			if (fix.valid.location) {

				currentLoc = fix.location;
	
				// Todo: there's some fuckery that needs to be done with GPS vs. inertial bearings
				// For now, just update it every fix
				bearingError = bearingSmoothing*(fix.heading() - DeadReckoner::getBearing()) + (1-bearingSmoothing)*bearingError;
				float speed_mps = fix.speed_metersph() / 3600.0;
				float heading_rad = (fix.heading() - bearingError) / 180.0 * M_PI;

				DeadReckoner::resetPositionReckoning(cos(heading_rad) * speed_mps, -sin(heading_rad) * speed_mps);

				telem_gpsFix(currentLoc.lat(), currentLoc.lon());
			}
		} else {
			Vector2f offset(DeadReckoner::horizontalX(), DeadReckoner::horizontalY());
			float bearing = atan2(offset.x(), offset.y()) / M_PI * 180.0;
			bearing += bearingError;
			float distanceM = offset.norm();
			//Serial.printf("Pos offset: %f, %f\n", offset.x(), offset.y());

			currentLoc = fix.location;
			currentLoc.OffsetBy(distanceM / 1000 / NeoGPS::Location_t::EARTH_RADIUS_KM, bearing);

			telem_gpsReckon(currentLoc.lat(), currentLoc.lon());
		}

		//If we're close enough to our target, move on to the next one in the buffer;
		if(currentLoc.DistanceKm(targetLoc)*1000 < target_dist){
			NeoGPS::Location_t buffer = targetLoc;
			targetLoc = waypoints.pop();
			waypoints.put(buffer);
		}
        bearingToTarget = currentLoc.BearingToDegrees(targetLoc);

		if(enableGPSNav){
			//setTargetBearing(bearingToTarget - bearingError);
		}
	}
	float getBearingError(){
		return bearingError;
	}

	float getBearingToTarget(){
		return bearingToTarget;
	}
}
