/*	WMM Subroutine library was tested in the following environments
 *
 *	1. Red Hat Linux  with GCC Compiler
 *	2. MS Windows XP with CodeGear C++ compiler
 *	3. Sun Solaris with GCC Compiler
 *
 *
 *      Revision Number: $Revision: 833 $
 *      Last changed by: $Author: awoods $
 *      Last changed on: $Date: 2012-04-16 14:01:51 -0600 (Mon, 16 Apr 2012) $
 *
 *
 */


#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE
#endif

//#ifndef EPOCHRANGE
//#define EPOCHRANGE (int)5
//#endif

#define READONLYMODE "r"
#define MAXLINELENGTH (1024)
#define NOOFPARAMS (15)
#define NOOFCOEFFICIENTS (7)


#define _DEGREE_NOT_FOUND (-2)
#define CALCULATE_NUMTERMS(N)    (N * ( N + 1 ) / 2 + N)

#ifndef WMMHEADER_H
#define WMMHEADER_H

#ifndef M_PI
#define M_PI    ((2)*(acos(0.0)))
#endif

#define RAD2DEG(rad)    ((rad)*(180.0L/M_PI))
#define DEG2RAD(deg)    ((deg)*(M_PI/180.0L))
#define ATanH(x)	    (0.5 * log((1 + x) / (1 - x)))


#define TRUE            ((int)1)
#define FALSE           ((int)0)




#define MAG_PS_MIN_LAT_DEGREE  -55 /* Minimum Latitude for  Polar Stereographic projection in degrees   */
#define MAG_PS_MAX_LAT_DEGREE  55  /* Maximum Latitude for Polar Stereographic projection in degrees     */
#define MAG_UTM_MIN_LAT_DEGREE -80.5  /* Minimum Latitude for UTM projection in degrees   */
#define MAG_UTM_MAX_LAT_DEGREE  84.5  /* Maximum Latitude for UTM projection in degrees     */

#define MAG_GEO_POLE_TOLERANCE  1e-5
#define MAG_USE_GEOID	1    /* 1 Geoid - Ellipsoid difference should be corrected, 0 otherwise */

/*
Data types and prototype declaration for
World Magnetic Model (WMM) subroutines.

July 28, 2009

manoj.c.nair@noaa.gov*/


typedef struct {
    double EditionDate;
    double epoch; //Base time of Geomagnetic model epoch (yrs)
    char ModelName[32];
    double *Main_Field_Coeff_G; // C - Gauss coefficients of main geomagnetic model (nT)
    double *Main_Field_Coeff_H; // C - Gauss coefficients of main geomagnetic model (nT)
    double *Secular_Var_Coeff_G; // CD - Gauss coefficients of secular geomagnetic model (nT/yr)
    double *Secular_Var_Coeff_H; // CD - Gauss coefficients of secular geomagnetic model (nT/yr)
    int nMax; // Maximum degree of spherical harmonic model
    int nMaxSecVar; //Maxumum degree of spherical harmonic secular model
    int SecularVariationUsed; //Whether or not the magnetic secular variation vector will be needed by program
    double CoefficientFileEndDate; //
} MAGtype_MagneticModel;

typedef struct {
    double a; /*semi-major axis of the ellipsoid*/
    double b; /*semi-minor axis of the ellipsoid*/
    double fla; /* flattening */
    double epssq; /*first eccentricity squared */
    double eps; /* first eccentricity */
    double re; /* mean radius of  ellipsoid*/
} MAGtype_Ellipsoid;

typedef struct {
    double lambda; // longitude
    double phi; // geodetic latitude
    double HeightAboveEllipsoid; // height above the ellipsoid (HaE)
    double HeightAboveGeoid; /* (height above the EGM96 geoid model ) */
    int UseGeoid;
} MAGtype_CoordGeodetic;

typedef struct {
    double lambda; /* longitude*/
    double phig; /* geocentric latitude*/
    double r; /* distance from the center of the ellipsoid*/
} MAGtype_CoordSpherical;

typedef struct {
    int Year;
    int Month;
    int Day;
    double DecimalYear; /* decimal years */
} MAGtype_Date;

/*typedef struct { 
    int MAG_Mercator;
    int MAG_LambertConformalConic;
    int MAG_PolarStereographic;
    int MAG_TransverseMercator;
} MAGtype_MapProjectionCode; 
 Deprecated, Will be removed in the next revision.*/

typedef struct {
    double *Pcup; /* Legendre Function */
    double *dPcup; /* Derivative of Legendre fcn */
} MAGtype_LegendreFunction;

typedef struct {
    double Bx; /* North */
    double By; /* East */
    double Bz; /* Down */
} MAGtype_MagneticResults;

typedef struct {
    double *RelativeRadiusPower; /* [earth_reference_radius_km / sph. radius ]^n  */
    double *cos_mlambda; /*cp(m)  - cosine of (m*spherical coord. longitude)*/
    double *sin_mlambda; /* sp(m)  - sine of (m*spherical coord. longitude) */
} MAGtype_SphericalHarmonicVariables;

typedef struct {
    double Decl; /* 1. Angle between the magnetic field vector and true north, positive east*/
    double Incl; /*2. Angle between the magnetic field vector and the horizontal plane, positive down*/
    double F; /*3. Magnetic Field Strength*/
    double H; /*4. Horizontal Magnetic Field Strength*/
    double X; /*5. Northern component of the magnetic field vector*/
    double Y; /*6. Eastern component of the magnetic field vector*/
    double Z; /*7. Downward component of the magnetic field vector*/
    double GV; /*8. The Grid Variation*/
    double Decldot; /*9. Yearly Rate of change in declination*/
    double Incldot; /*10. Yearly Rate of change in inclination*/
    double Fdot; /*11. Yearly rate of change in Magnetic field strength*/
    double Hdot; /*12. Yearly rate of change in horizontal field strength*/
    double Xdot; /*13. Yearly rate of change in the northern component*/
    double Ydot; /*14. Yearly rate of change in the eastern component*/
    double Zdot; /*15. Yearly rate of change in the downward component*/
    double GVdot; /*16. Yearly rate of change in grid variation*/
} MAGtype_GeoMagneticElements;

typedef struct {
    int NumbGeoidCols; /* 360 degrees of longitude at 15 minute spacing */
    int NumbGeoidRows; /* 180 degrees of latitude  at 15 minute spacing */
    int NumbHeaderItems; /* min, max lat, min, max long, lat, long spacing*/
    int ScaleFactor; /* 4 grid cells per degree at 15 minute spacing  */
    float *GeoidHeightBuffer;
    int NumbGeoidElevs;
    int Geoid_Initialized; /* indicates successful initialization */
    int UseGeoid; /*Is the Geoid being used?*/
} MAGtype_Geoid;

typedef struct {
    char Longitude[40];
    char Latitude[40];
} MAGtype_CoordGeodeticStr;

typedef struct {
    double Easting; /* (X) in meters*/
    double Northing; /* (Y) in meters */
    int Zone; /*UTM Zone*/
    char HemiSphere;
    double CentralMeridian;
    double ConvergenceOfMeridians;
    double PointScale;
} MAGtype_UTMParameters;

enum PARAMS {
    SHDF,
    MODELNAME,
    PUBLISHER,
    RELEASEDATE,
    DATACUTOFF,
    MODELSTARTYEAR,
    MODELENDYEAR,
    EPOCH,
    INTSTATICDEG,
    INTSECVARDEG,
    EXTSTATICDEG,
    EXTSECVARDEG,
    GEOMAGREFRAD,
    NORMALIZATION,
    SPATBASFUNC
};

enum COEFFICIENTS {
    IE,
    N,
    M,
    GNM,
    HNM,
    DGNM,
    DHNM
};

enum YYYYMMDD {
    YEAR,
    MONTH,
    DAY
};

/*Prototypes */

/*Wrapper Functions*/
int MAG_Geomag(MAGtype_Ellipsoid Ellip,
        MAGtype_CoordSpherical CoordSpherical,
        MAGtype_CoordGeodetic CoordGeodetic,
        MAGtype_MagneticModel *TimedMagneticModel,
        MAGtype_GeoMagneticElements *GeoMagneticElements);

int MAG_Grid(MAGtype_CoordGeodetic minimum,
        MAGtype_CoordGeodetic maximum,
        double step_size,
        double altitude_step_size,
        double time_step,
        MAGtype_MagneticModel *MagneticModel,
        MAGtype_Geoid *geoid,
        MAGtype_Ellipsoid Ellip,
        MAGtype_Date StartDate,
        MAGtype_Date EndDate,
        int ElementOption,
        int PrintOption,
        char *OutputFile);

int MAG_robustReadMagneticModel_Large(char *filename, char* filenameSV, MAGtype_MagneticModel **MagneticModel, int array_size);

int MAG_robustReadMagModels(char *filename, MAGtype_MagneticModel *(*magneticmodels)[], int array_size);

int MAG_SetDefaults(MAGtype_Ellipsoid *Ellip, MAGtype_Geoid *Geoid);

/*User Interface*/

void MAG_Error(int control);

char MAG_GeomagIntroduction_WMM(MAGtype_MagneticModel *MagneticModel, char *VersionDate);

char MAG_GeomagIntroduction_EMM(MAGtype_MagneticModel *MagneticModel, char *VersionDate);

int MAG_GetUserGrid(MAGtype_CoordGeodetic *minimum,
        MAGtype_CoordGeodetic *maximum,
        double *step_size,
        double *a_step_size,
        double *step_time,
        MAGtype_Date *StartDate,
        MAGtype_Date *EndDate,
        int *ElementOption,
        int *PrintOption,
        char *OutputFile,
        MAGtype_Geoid *Geoid);

int MAG_GetUserInput(MAGtype_MagneticModel *MagneticModel,
        MAGtype_Geoid *Geoid,
        MAGtype_CoordGeodetic *CoordGeodetic,
        MAGtype_Date *MagneticDate);

void MAG_PrintUserData(MAGtype_GeoMagneticElements GeomagElements,
        MAGtype_CoordGeodetic SpaceInput,
        MAGtype_Date TimeInput,
        MAGtype_MagneticModel *MagneticModel,
        MAGtype_Geoid *Geoid);

int MAG_ValidateDMSstringlat(char *input, char *Error);

int MAG_ValidateDMSstringlong(char *input, char *Error);

int MAG_Warnings(int control, double value, MAGtype_MagneticModel *MagneticModel);

/*Memory and File Processing*/

MAGtype_LegendreFunction *MAG_AllocateLegendreFunctionMemory(int NumTerms);

MAGtype_MagneticModel *MAG_AllocateModelMemory(int NumTerms);

MAGtype_SphericalHarmonicVariables *MAG_AllocateSphVarMemory(int nMax);

void MAG_AssignHeaderValues(MAGtype_MagneticModel *model, char values[][MAXLINELENGTH]);

void MAG_AssignMagneticModelCoeffs(MAGtype_MagneticModel *Assignee, MAGtype_MagneticModel *Source, int nMax, int nMaxSecVar);

int MAG_FreeMemory(MAGtype_MagneticModel *MagneticModel, MAGtype_MagneticModel *TimedMagneticModel, MAGtype_LegendreFunction *LegendreFunction);

int MAG_FreeLegendreMemory(MAGtype_LegendreFunction *LegendreFunction);

int MAG_FreeMagneticModelMemory(MAGtype_MagneticModel *MagneticModel);

int MAG_FreeSphVarMemory(MAGtype_SphericalHarmonicVariables *SphVar);

void MAG_PrintWMMFormat(char *filename, MAGtype_MagneticModel *MagneticModel);

void MAG_PrintEMMFormat(char *filename, char *filenameSV, MAGtype_MagneticModel *MagneticModel);

int MAG_readMagneticModel(char *filename, MAGtype_MagneticModel *MagneticModel);

int MAG_readMagneticModel_Large(char *filename, char *filenameSV, MAGtype_MagneticModel *MagneticModel);

int MAG_readMagneticModel_SHDF(char *filename, MAGtype_MagneticModel *(*magneticmodels)[], int array_size);

int MAG_swab_type();

char *MAG_Trim(char *str);

float MAG_FloatSwap(float f);

/*Conversions, Transformations, and other Calculations*/

int MAG_CalculateGeoMagneticElements(MAGtype_MagneticResults *MagneticResultsGeo, MAGtype_GeoMagneticElements *GeoMagneticElements);

int MAG_CalculateSecularVariationElements(MAGtype_MagneticResults MagneticVariation, MAGtype_GeoMagneticElements *MagneticElements);

int MAG_CalculateGridVariation(MAGtype_CoordGeodetic location, MAGtype_GeoMagneticElements *elements);

int MAG_DateToYear(MAGtype_Date *Calendar_Date, char *Error);

void MAG_DegreeToDMSstring(double DegreesOfArc, int UnitDepth, char *DMSstring);

void MAG_DMSstringToDegree(char *DMSstring, double *DegreesOfArc);

int MAG_GeodeticToSpherical(MAGtype_Ellipsoid Ellip, MAGtype_CoordGeodetic CoordGeodetic, MAGtype_CoordSpherical *CoordSpherical);

int MAG_GetTransverseMercator(MAGtype_CoordGeodetic CoordGeodetic, MAGtype_UTMParameters *UTMParameters);

int MAG_GetUTMParameters(double Latitude,
        double Longitude,
        int *Zone,
        char *Hemisphere,
        double *CentralMeridian);

int MAG_isNaN(double d);

int MAG_RotateMagneticVector(MAGtype_CoordSpherical,
        MAGtype_CoordGeodetic CoordGeodetic,
        MAGtype_MagneticResults MagneticResultsSph,
        MAGtype_MagneticResults *MagneticResultsGeo);

void MAG_SphericalToCartesian(MAGtype_CoordSpherical CoordSpherical, double *x, double *y, double *z);

void MAG_TMfwd4(double Eps, double Epssq, double K0R4, double K0R4oa,
        double Acoeff[], double Lam0, double K0, double falseE,
        double falseN, int XYonly, double Lambda, double Phi,
        double *X, double *Y, double *pscale, double *CoM);  //Rewrite this?

int MAG_YearToDate(MAGtype_Date *Date);


/*Spherical Harmonics*/

int MAG_AssociatedLegendreFunction(MAGtype_CoordSpherical CoordSpherical, int nMax, MAGtype_LegendreFunction *LegendreFunction);

int MAG_CheckGeographicPole(MAGtype_CoordGeodetic *CoordGeodetic);

int MAG_ComputeSphericalHarmonicVariables(MAGtype_Ellipsoid Ellip,
        MAGtype_CoordSpherical CoordSpherical,
        int nMax,
        MAGtype_SphericalHarmonicVariables * SphVariables);

int MAG_PcupHigh(double *Pcup, double *dPcup, double x, int nMax);

int MAG_PcupLow(double *Pcup, double *dPcup, double x, int nMax);

int MAG_SecVarSummation(MAGtype_LegendreFunction *LegendreFunction,
        MAGtype_MagneticModel *MagneticModel,
        MAGtype_SphericalHarmonicVariables SphVariables,
        MAGtype_CoordSpherical CoordSpherical,
        MAGtype_MagneticResults *MagneticResults);

int MAG_SecVarSummationSpecial(MAGtype_MagneticModel *MagneticModel,
        MAGtype_SphericalHarmonicVariables SphVariables,
        MAGtype_CoordSpherical CoordSpherical,
        MAGtype_MagneticResults *MagneticResults);

int MAG_Summation(MAGtype_LegendreFunction *LegendreFunction,
        MAGtype_MagneticModel *MagneticModel,
        MAGtype_SphericalHarmonicVariables SphVariables,
        MAGtype_CoordSpherical CoordSpherical,
        MAGtype_MagneticResults *MagneticResults);

int MAG_SummationSpecial(MAGtype_MagneticModel *MagneticModel,
        MAGtype_SphericalHarmonicVariables SphVariables,
        MAGtype_CoordSpherical CoordSpherical,
        MAGtype_MagneticResults *MagneticResults);

int MAG_TimelyModifyMagneticModel(MAGtype_Date UserDate, MAGtype_MagneticModel *MagneticModel, MAGtype_MagneticModel *TimedMagneticModel);

/*Geoid*/

int MAG_InitializeGeoid(MAGtype_Geoid *Geoid);



int MAG_ConvertGeoidToEllipsoidHeight(MAGtype_CoordGeodetic *CoordGeodetic, MAGtype_Geoid *Geoid);
/*
 * The function Convert_Geoid_To_Ellipsoid_Height converts the specified WGS84
 * geoid height at the specified geodetic coordinates to the equivalent
 * ellipsoid height, using the EGM96 gravity model.
 *
 *    Latitude            : Geodetic latitude in radians           (input)
 *    Longitude           : Geodetic longitude in radians          (input)
 *    Geoid_Height        : Geoid height, in meters                (input)
 *    Ellipsoid_Height    : Ellipsoid height, in meters.           (output)
 *
 */

int MAG_GetGeoidHeight(double Latitude, double Longitude, double *DeltaHeight, MAGtype_Geoid *Geoid);
/*
 * The private function Get_Geoid_Height returns the height of the
 * WGS84 geiod above or below the WGS84 ellipsoid,
 * at the specified geodetic coordinates,
 * using a grid of height adjustments from the EGM96 gravity model.
 *
 *    Latitude            : Geodetic latitude in radians           (input)
 *    Longitude           : Geodetic longitude in radians          (input)
 *    DeltaHeight         : Height Adjustment, in meters.          (output)
 *
 */






#endif /*WMMHEADER_H*/
