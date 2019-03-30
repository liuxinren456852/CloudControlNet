#ifndef GEOTRAN_H
#define GEOTRAN_H

#include <ogr_spatialref.h>

#include <vector>
#include <list>

using namespace std;

class GeoTransform
{
public:

	//Convert [Longitude,Latitude,Elevation] Coordinate to [X,Y,Z] Coordinate by UTM projection under WGS84 System
	int BLH2XYZ_WGS84(const std::vector<double> &BLH_coord, std::vector<double> &XYZ_coord)
	{
		OGRSpatialReference *RefSource = new OGRSpatialReference;
		RefSource->SetWellKnownGeogCS("WGS84");

		OGRSpatialReference *RefTarget = new OGRSpatialReference;
		RefTarget = RefSource->CloneGeogCS();

		int utmzone = BLH_coord[1] / 6 + 31;  //six degree zone

		RefTarget->SetProjCS("UTM(WGS84) in northern hemisphere.");
		RefTarget->SetUTM(utmzone, TRUE);
		OGRCoordinateTransformation *poTransform = OGRCreateCoordinateTransformation(RefSource, RefTarget);

		double tempX = BLH_coord[1];
		double tempY = BLH_coord[0];
		double tempZ = BLH_coord[2];

		poTransform->Transform(1, &tempX, &tempY, &tempZ);

		XYZ_coord.resize(3);

		XYZ_coord[0] = tempX;
		XYZ_coord[1] = tempY;
		XYZ_coord[2] = tempZ;

		return utmzone;
	}

	//Convert [X,Y,Z] Coordinate to [Longitude,Latitude,Elevation] Coordinate by UTM inverse projection under WGS84 System with given UTM zone number.
	void XYZ2BLH_WGS84(const std::vector<double> &XYZ_coord, const int &utmzone, std::vector<double> &BLH_coord)
	{
		OGRSpatialReference *RefSource = new OGRSpatialReference;
		RefSource->SetWellKnownGeogCS("WGS84");
		RefSource->SetProjCS("UTM(WGS84) in northern hemisphere.");
		RefSource->SetUTM(utmzone, TRUE);

		OGRSpatialReference *RefTarget = new OGRSpatialReference;
		RefTarget = RefSource->CloneGeogCS();

		OGRCoordinateTransformation *poTranform = OGRCreateCoordinateTransformation(RefSource, RefTarget);

		double tempx = XYZ_coord[0];
		double tempy = XYZ_coord[1];
		double tempz = XYZ_coord[2];

		poTranform->Transform(1, &tempx, &tempy, NULL);
		BLH_coord.resize(3);
		BLH_coord[1] = tempx; //Longitude
		BLH_coord[0] = tempy; //Latitude
		BLH_coord[2] = tempz; //Elevation
	}

protected:

private:
	
};


#endif //GEOTRAN_H