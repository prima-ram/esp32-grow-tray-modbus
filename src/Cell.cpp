#include "Cell.h"
#include <math.h>

Cell::Cell(int hxDout, int hxSck)
		: hx(hxDout, hxSck)
{
	// Initialize
	hx.begin();
}

void Cell::SetFactor(float factor)
{
	if (isnan(factor))
	{
		factor = 1.0f;
	}
	hx.setCalFactor(factor);
}

void Cell::SetOffset(long offset)
{
	hx.setTareOffset(offset);
}

uint8_t Cell::Update()
{
	return hx.update();
}

long Cell::Read()
{
	return hx.getData();
}

void Cell::StartTare()
{
	hx.tareNoDelay();
}

bool Cell::TareComplete() const
{
	return const_cast<HX711_ADC &>(hx).getTareStatus();
}

long Cell::TareOffset() const
{
	return const_cast<HX711_ADC &>(hx).getTareOffset();
}

float Cell::Calibrate(float known_grams)
{
	if (isnan(known_grams))
	{
		known_grams = 1.0f;
	}
	return hx.getNewCalibration(known_grams);
}

void Cell::StartRefreshDataSet()
{
	hx.resetSamplesIndex();
}

bool Cell::RefreshDataSetComplete() const
{
	return const_cast<HX711_ADC &>(hx).getDataSetStatus();
}
