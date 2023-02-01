namespace Pilot {

	void pilotSetup();
	void pilotStart();
	void pilotLoop();

	// Set any of these values to NaN to disable its usage.
	struct targets_t {
		float airspeed = 8;
		float altitude = 360; // High enough above Oberlin to not hit anything
		float bearing = 180; // The direction we pointed when calibration happened
	};
	extern targets_t targets;
}

// returns a value of s - t between -180 and 180 degrees.
float normAngle(float x);
float angleDiff(float s, float t);
