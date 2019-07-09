#ifndef GEOTRAN_H
#define GEOTRAN_H

//#include <ogr_spatialref.h>
//#include "cpl_conv.h"

#include <vector>
#include <list>

using namespace std;

class GeoTransform
{
public:

#if 0
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
	void XYZ2BLH_WGS84(const std::vector<double> &XYZ_coord, const int utmzone, std::vector<double> &BLH_coord)
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



	void XYZ2BLH_ENG(const std::vector<double> &XYZ_coord,float centerlong, std::vector<double> &BLH_coord)
	{
		//CPLSetConfigOption("GDAL_DATA", "F:\\4_softwares\\gdaldata");
		OGRSpatialReference *RefSource = new OGRSpatialReference;
		RefSource->SetWellKnownGeogCS("EPSG:4490");//CGCS2000
		
		//Other Common Used CS's code in EPSG
		//spatialReference.importFromEPSG(4326);//WGS84
		//spatialReference.importFromEPSG(4214);//BeiJing54
		//spatialReference.importFromEPSG(4610);//XIAN80
		//spatialReference.importFromEPSG(4490);//CGCS2000

		RefSource->SetProjCS("CGCS2000/UTM");
		RefSource->SetTM(0, centerlong, 1.0, 500000, 0); // centerlong 104 24' here,  104 + 24.0 / 60   //Universal
		
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


	void BLH2XYZ_CGCS(const std::vector<double> &BLH_coord, float centerlong, float proj_surface_h_eng, std::vector<double> &XYZ_coord)
	{
		//CGCS2000 Parameters
		const double a = 6378137.0;
		const double b = 6356752.314;
		//const double R = 6378245.0;
		const double shift = 500000.0;

		//CPLSetConfigOption("GDAL_DATA", "F:\\4_softwares\\gdaldata");
		OGRSpatialReference *RefSource = new OGRSpatialReference;
		RefSource->SetWellKnownGeogCS("EPSG:4490");//CGCS2000

		//Other Common Used CS's code in EPSG
		//spatialReference.importFromEPSG(4326);//WGS84
		//spatialReference.importFromEPSG(4214);//BeiJing54
		//spatialReference.importFromEPSG(4610);//XIAN80
		//spatialReference.importFromEPSG(4490);//CGCS2000

		
		OGRSpatialReference *RefTarget = new OGRSpatialReference;
		RefTarget = RefSource->CloneGeogCS();
		RefTarget->SetProjCS("CGCS2000/UTM");
		RefTarget->SetTM(0, centerlong, 1.0, (int)shift, 0); // centerlong 104 24' here,  104 + 24.0 / 60   //Universal 0.9996

		OGRCoordinateTransformation *poTranform = OGRCreateCoordinateTransformation(RefSource, RefTarget);

		double tempX = BLH_coord[1];
		double tempY = BLH_coord[0];
		double tempZ = BLH_coord[2];
		double R = MeanRofEarth(a, b, tempY);
		cout << "R is " << R << endl;
		poTranform->Transform(1, &tempX, &tempY, &tempZ);

		XYZ_coord.resize(3);
		
		XYZ_coord[0] = (tempX-shift)*(1 + proj_surface_h_eng/R) + shift;
		XYZ_coord[1] = tempY*(1 + proj_surface_h_eng / R) ;
		XYZ_coord[2] = tempZ;
	}
	//Our requirement;
	//ENG XYZ ->> CGCS BLH ->> UTM XYZ;
	//XYZ2BLH_ENG(XYZ_eng_coord,centerlong,BLH_coord);    
	//BLH2XYZ_WGS84(BLH_coord,XYZ_utm_coord);

	//Try 4DOF Coordinate Transformation. 

#endif

protected:
	double MeanRofEarth(double a, double b, double B)
	{
		const double pi_ = 3.141592654;
		B = B / 180 * pi_;
		double e0 = sqrt((a*a - b*b) / a / a);
		//double ep = e0*a/b;
		double W = sqrt(1 - e0*e0*sin(B)*sin(B));
		double N = a / W;
		double M = a*(1 - e0*e0) / W / W / W;
		double R = sqrt(N*M);
		return R;
	}

private:
	
};


#endif //GEOTRAN_H