#pragma once

#include <HX711_ADC.h>

class Cell
{
private:
	HX711_ADC hx;

public:
	Cell(int hxDout, int hxSck);
	/**
	 * Update the measurement of the Cell
	 */
	uint8_t Update();

	void SetFactor(float factor);
	void SetOffset(long offset);
	/**
	 * Get average readings passing a number of readings
	 */
	long Read();
	void StartTare();
	bool TareComplete() const;
	long TareOffset() const;
	/**
	 * Calibrate cell giving the actual grams on the cell
	 */
	float Calibrate(float grams);

	// Start a non-blocking data set refresh. Regular Update() calls will refill it.
	void StartRefreshDataSet();

	// Return true once the refreshed data set has been fully refilled.
	bool RefreshDataSetComplete() const;
};
