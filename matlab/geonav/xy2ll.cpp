/*==========================================================
 * xy2ll.cpp for conversion to MEX file for MATLAB
 *========================================================*/

#include "mex.h"
#include "../../include/geonav_transform/navsat_conversions.h"
/* Computational routine */
void xy2ll(double x, double y, double origin_lat, double origin_lon,
	   double *lat, double *lon)
{
  std::string outmzone, utmzone;
  double outmy, outmx, utmx, utmy;

  GeonavTransform::NavsatConversions::LLtoUTM(origin_lat,origin_lon,outmy,outmx,outmzone);
  utmy = outmy+y;
  utmx = outmx+x;
  double tlat, tlon;
  GeonavTransform::NavsatConversions::UTMtoLL(utmy,utmx,outmzone,tlat,tlon);
  *lat=tlat;
  *lon=tlon;


}


/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
  /* Inputs */
  double x;
  double y;
  double origin_lat;
  double origin_lon;
  /* Outputs */
  double *lat;
  double *lon;

  /* check for proper number of arguments */
    if(nrhs!=4) {
        mexErrMsgIdAndTxt("geonav_conversions:xy2ll:nrhs","Four inputs required.");
    }
    if(nlhs!=2) {
        mexErrMsgIdAndTxt("geonav_conversions:xy2ll:nlhs","Two outputs required.");
    }
    /* make sure the all inputs argument are scalar */
    int ii = 0;
    for (ii=0; ii <= 3; ii++)
      {
	if( !mxIsDouble(prhs[ii]) || 
	    mxIsComplex(prhs[ii]) ||
	    mxGetNumberOfElements(prhs[ii])!=1 ) 
	  {
	    mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notScalar","Input multiplier must be a scalar.");
	  }
      }
    /* get the value of the scalar inputs  */
    x = mxGetScalar(prhs[0]);
    y = mxGetScalar(prhs[1]);
    origin_lat = mxGetScalar(prhs[2]);
    origin_lon = mxGetScalar(prhs[3]);


    /* create the output */
    plhs[0] = mxCreateDoubleScalar(0);
    plhs[1] = mxCreateDoubleScalar(0);

    lat = mxGetPr(plhs[0]);
    lon = mxGetPr(plhs[1]);

    /* call the computational routine */
    xy2ll(x,y,origin_lat,origin_lon,lat,lon);
}
