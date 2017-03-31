/*==========================================================
 * ll2xy.cpp for conversion to MEX file for MATLAB
 *========================================================*/

#include "mex.h"
#include "../../include/geonav_transform/navsat_conversions.h"
/* Computational routine */
void ll2xy(double lat, double lon, double origin_lat, double origin_lon,
	   double *x, double *y)
{
  std::string outmzone, utmzone;
  double outmy, outmx, utmx, utmy;
  GeonavTransform::NavsatConversions::LLtoUTM(origin_lat,origin_lon,outmy,outmx,outmzone);
  GeonavTransform::NavsatConversions::LLtoUTM(lat,lon,utmy,utmx,utmzone);
  *x = utmx-outmx;
  *y = utmy-outmy;

}


/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
  /* Inputs */
  double lat;
  double lon;
  double origin_lat;
  double origin_lon;
  /* Outputs */
  double *x;
  double *y;

  /* check for proper number of arguments */
    if(nrhs!=4) {
        mexErrMsgIdAndTxt("geonav_conversions:ll2xy:nrhs","Four inputs required.");
    }
    if(nlhs!=2) {
        mexErrMsgIdAndTxt("geonav_conversions:ll2xy:nlhs","Two outputs required.");
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
    lat = mxGetScalar(prhs[0]);
    lon = mxGetScalar(prhs[1]);
    origin_lat = mxGetScalar(prhs[2]);
    origin_lon = mxGetScalar(prhs[3]);


    /* create the output */
    plhs[0] = mxCreateDoubleScalar(0);
    plhs[1] = mxCreateDoubleScalar(0);

    x = mxGetPr(plhs[0]);
    y = mxGetPr(plhs[1]);

    /* call the computational routine */
    ll2xy(lat,lon,origin_lat,origin_lon,x,y);
}
