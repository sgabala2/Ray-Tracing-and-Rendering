#include <algorithm>
#include "image.h"
#include <stdlib.h>
#include <math.h>
#include <Util/exceptions.h>

using namespace Util;
using namespace Image;

//////////////////////
// Helper Functions //
//////////////////////

/** Function to make sure RGB color chanels stay within range [0, 255] */
float clampRGB( float value )
{
	if(value > 255) { return 255;} 
	if(value < 0) { return 0; }

	return value;
}

/** Function to compute the mean average luminance of an image */
float meanImageLuminance(Image32 im)
{
	float totalIntensity = 0;
	int width = im.width();
	int height = im.height();

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			Pixel32 p1 = im.operator()(x, y);
			totalIntensity += (p1.r * 0.3) + (p1.g * 0.59) + (p1.b * 0.11);
		}
	}

	return totalIntensity / (width * height);
}

/////////////
// Image32 //
/////////////
Image32 Image32::addRandomNoise( double noise ) const
{

	Image32 outputImage;
	outputImage.setSize(width(), height());

	float rnoise;
	float range = 255 * noise;

	if (noise >= 0 && noise <= 1) {
		for (int y = 0; y < height(); y++) {
			for (int x = 0; x < width(); x++) {				
				Pixel32 p1 = operator()(x, y);
				
				//generate random noise in range [-noise, noise]
				rnoise = (rand() % (int)(2*range + 1)) - range;
			
				//add noise to pixel 
				p1.r = clampRGB(p1.r + rnoise);
				p1.g = clampRGB(p1.g + rnoise);
				p1.b = clampRGB(p1.b + rnoise);

				//copy new value to destination image
				outputImage.operator()(x, y) = p1;
			}
		}
	}

	return outputImage;
}

Image32 Image32::brighten( double brightness ) const
{
	/////////////////////////
	// Do brightening here //
	/////////////////////////

	Image32 outputImage;
	outputImage.setSize(width(), height());

	for (int y = 0; y < height(); y++) {
		for (int x = 0; x < width(); x++) {				
			Pixel32 p1 = operator()(x, y); 
			
			//add brightness by mutiplying each color chanel by factor
			p1.r = clampRGB(p1.r * brightness);
			p1.g = clampRGB(p1.g * brightness);
			p1.b = clampRGB(p1.b * brightness);

			outputImage(x, y) = p1;
		}
	}

	return outputImage;
}

Image32 Image32::luminance( void ) const
{
	//////////////////////////////////
	// Compute luminance image here //
	//////////////////////////////////

	Image32 outputImage;
	outputImage.setSize(width(), height());

	for (int y = 0; y < height(); y++) {
		for (int x = 0; x < width(); x++) {				
			Pixel32 p1 = operator()(x, y);

			//comute the weighted average of the pixel
			float Lp = clampRGB((p1.r * 0.3) + (p1.g * 0.59) + (p1.b * 0.11));

			p1.r = Lp;
			p1.g = Lp;
			p1.b = Lp;

			outputImage(x, y) = p1;
		}
	}

	return outputImage;
}

Image32 Image32::contrast( double contrast ) const
{
	//////////////////////////////////
	// Do contrast enhancement here //
	//////////////////////////////////

	Image32 outputImage;
	outputImage.setSize(width(), height());

	//compute mean image luminance -- averaged over all pixels
	float Lp = meanImageLuminance(*this);
	
	// scale deviation from 𝐋̅ for each pixel component, must clamp as well
	for (int y = 0; y < height(); y++) {
		for (int x = 0; x < width(); x++) {				
			Pixel32 p1 = operator()(x, y);

			p1.r = clampRGB((p1.r - Lp) * contrast + Lp);
			p1.g = clampRGB((p1.g - Lp) * contrast + Lp);
			p1.b = clampRGB((p1.b - Lp) * contrast + Lp);

			outputImage(x, y) = p1;
		}
	}

	return outputImage;
}

Image32 Image32::saturate( double saturation ) const
{
	////////////////////////////////////
	// Do saturation enhancement here //
	////////////////////////////////////

	Image32 outputImage;
	outputImage.setSize(width(), height());

	for (int y = 0; y < height(); y++) {
		for (int x = 0; x < width(); x++) {				
			Pixel32 p1 = operator()(x, y);

			//weighted average of the pixel
			float Lp = (p1.r * 0.3) + (p1.g * 0.59) + (p1.b * 0.11);

			p1.r = clampRGB((p1.r - Lp) * saturation + Lp);
			p1.g = clampRGB((p1.g - Lp) * saturation + Lp);
			p1.b = clampRGB((p1.b - Lp) * saturation + Lp);

			outputImage(x, y) = p1;
		}
	}

	return outputImage;
}

Image32 Image32::quantize( int bits ) const
{
	//////////////////////////
	// Do quantization here //
	//////////////////////////

	Image32 outputImage;
	outputImage.setSize(width(), height());

	int q = pow(2, bits);

	for (int y = 0; y < height(); y++) {
		for (int x = 0; x < width(); x++) {				
			Pixel32 p1 = operator()(x, y);

			p1.r = clampRGB(floor((p1.r/256.0) * q) * (255.0 / (q-1)));
			p1.g = clampRGB(floor((p1.g/256.0) * q) * (255.0 / (q-1)));
			p1.b = clampRGB(floor((p1.b/256.0) * q) * (255.0 / (q-1)));

			outputImage(x, y) = p1;
		}
	}

	return outputImage;
}

Image32 Image32::randomDither( int bits ) const
{
	//////////////////////////////
	// Do random dithering here //
	//////////////////////////////

	Image32 outputImage;
	outputImage.setSize(width(), height());

	int q = pow(2, bits);

	for (int y = 0; y < height(); y++) {
		for (int x = 0; x < width(); x++) {				
			Pixel32 p1 = operator()(x, y);

			//generating randome noise to add in range [-1, 1]
			float noise = (rand() % (int)(2*(255.0) + 1)) - (255.0);
			int q2 = noise / pow(2, bits);

			p1.r = clampRGB(floor(((p1.r + q2)/256.0) * q * (255.0 / (q-1))));
			p1.g = clampRGB(floor(((p1.g + q2)/256.0) * q * (255.0 / (q-1))));
			p1.b = clampRGB(floor(((p1.b + q2)/256.0) * q * (255.0 / (q-1))));

			outputImage(x, y) = p1;
		}
	}

	return outputImage;
}

Image32 Image32::orderedDither2X2( int bits ) const
{
	///////////////////////////////
	// Do ordered dithering here //
	///////////////////////////////
	Image32 outputImage;
	outputImage.setSize(width(), height());

	// 2x2 matrix stores patterns of thresholds 
	float dith[2][2] = {{0.25, 0.75}, {1, 0.5}};
	int q = pow(2, bits);
	int range = 255/(255/(256/q));

	for (int y = 0; y < height(); y++) {
		for (int x = 0; x < width(); x++) {				
			Pixel32 p1 = operator()(x, y);

			//locate the index in the matrix
			int i = x % 2;
			int j = y % 2;

			//get fractional component
			float cR = (p1.r / 256.0) * (q - 1);
			float cG = (p1.g / 256.0) * (q - 1);
			float cB = (p1.b / 256.0) * (q - 1);

			//round up or down depending on error
			float tempR = cR - floor(cR) > dith[i][j] ? ceil(cR) : floor(cR);
			float tempG = cG - floor(cG) > dith[i][j] ? ceil(cG) : floor(cG);
			float tempB = cB - floor(cB) > dith[i][j] ? ceil(cB) : floor(cB);

			//clamp
			p1.r = clampRGB(static_cast<int>(tempR) * range);
			p1.g = clampRGB(static_cast<int>(tempG) * range);
			p1.b = clampRGB(static_cast<int>(tempB) * range);

			outputImage(x, y) = p1;
		}
	}

	return outputImage;
}

Image32 Image32::floydSteinbergDither( int bits ) const
{
	///////////////////////////////////////
	// Do Floyd-Steinberg dithering here //
	///////////////////////////////////////
	Image32 outputImage = Image32();
	outputImage.setSize(width(), height()) ;
	outputImage = *this;

	int q = pow(2, bits);
	int f = 256 / q;
	int range = 255 / f;

	for (int y = 0; y < height(); y++) {
		for (int x = 0; x < width(); x++) {				
			Pixel32 p1 = outputImage(x, y);
			
			int eR = p1.r - p1.r / f * (255 / range);
			int eG = p1.g - p1.g / f * (255 / range);
			int eB = p1.b - p1.b / f * (255 / range);

			p1.r = p1.r / f * (255 / range);
			p1.g = p1.g / f * (255 / range);
			p1.b = p1.b / f * (255 / range);
			outputImage(x,y) = p1;

			//error difusion dithering
			if(x+1 < width()) {
				auto p1 = outputImage(x+1, y);
				p1.r = clampRGB(static_cast<int>(p1.r + eR * (7/16.0)));
				p1.g = clampRGB(static_cast<int>(p1.g + eG * (7/16.0)));
				p1.b = clampRGB(static_cast<int>(p1.b + eB * (7/16.0)));
				outputImage(x+1, y) = p1;
			}

			if(y+1 < height()) {
				if(x > 0) {
					auto p1 = outputImage(x-1, y+1);
					p1.r = clampRGB(static_cast<int>(p1.r + eR * (3/16.0)));
					p1.g = clampRGB(static_cast<int>(p1.g + eG * (3/16.0)));
					p1.b = clampRGB(static_cast<int>(p1.b + eB * (3/16.0)));
					outputImage(x-1, y+1) = p1;
				}

				if(x+1 < width()) {
					auto p1 = outputImage(x+1, y+1);
					p1.r = clampRGB(static_cast<int>(p1.r + eR * (1/16.0)));
					p1.g = clampRGB(static_cast<int>(p1.g + eG * (1/16.0)));
					p1.b = clampRGB(static_cast<int>(p1.b + eB * (1/16.0)));
					outputImage(x+1, y+1) = p1;
				}

				auto p1 = outputImage(x, y+1);
				p1.r = clampRGB(static_cast<int>(p1.r + eR * (5/16.0)));
				p1.g = clampRGB(static_cast<int>(p1.g + eG * (5/16.0)));
				p1.b = clampRGB(static_cast<int>(p1.b + eB * (5/16.0)));
				outputImage(x, y+1) = p1;
			}
		}
	}

	return outputImage;
}

Image32 Image32::blur3X3( void ) const
{
	//////////////////////
	// Do blurring here //
	//////////////////////

	Image32 outputImage = Image32();
	outputImage.setSize(width(), height()) ;
	outputImage = *this;

	double alpha = 1/16.0;
	double beta = 2/16.0;
	double gamma = 4/16.0;

	for (int y = 0; y < height(); y++) {
		for (int x = 0; x < width(); x++) {				
			Pixel32 p1 = outputImage(x, y);

			float newR = p1.r * gamma;
			float newG = p1.g * gamma;
			float newB = p1.b * gamma;

			if (x+1 < width()) {
				auto p1 = outputImage(x+1, y);
				newR += p1.r * beta;
				newG += p1.g * beta;
				newB += p1.b * beta;

				if(y+1 < height()) {
					auto p1 = outputImage(x+1, y+1);
					newR += p1.r * alpha;
					newG += p1.g * alpha;
					newB += p1.b * alpha;
				}

				if(y-1 > 0) {
					auto p1 = outputImage(x+1, y-1);
					newR += p1.r * alpha;
					newG += p1.g * alpha;
					newB += p1.b * alpha;

				}

			}

			if (x-1 > 0) {
				auto p1 = outputImage(x-1, y);
				newR += p1.r * beta;
				newG += p1.g * beta;
				newB += p1.b * beta;

				if(y+1 < height()) {
					newR += p1.r * alpha;
					newG += p1.g * alpha;
					newB += p1.b * alpha;
				}

				if(y-1 > 0) {
					newR += p1.r * alpha;
					newG += p1.g * alpha;
					newB += p1.b * alpha;
				}
			
			}

			if(y+1 < height()) {
				auto p1 = outputImage(x, y+1);
				newR += p1.r * beta;
				newG += p1.g * beta;
				newB += p1.b * beta;
			}

			if(y-1 > 0) {
				auto p1 = outputImage(x, y-1);
				newR += p1.r * beta;
				newG += p1.g * beta;
				newB += p1.b * beta;
			}

			p1.r = clampRGB(newR);
			p1.g = clampRGB(newG);
			p1.b = clampRGB(newB);

			outputImage(x, y) = p1;
		}
	}

	return outputImage;
}

Image32 Image32::edgeDetect3X3( void ) const
{
	////////////////////////////
	// Do edge detection here //
	////////////////////////////
	Image32 outputImage = Image32();
	outputImage.setSize(width(), height()) ;
	outputImage = *this;

	double alpha = -1.0;
	double beta = 8;

	for (int y = 0; y < height(); y++) {
		for (int x = 0; x < width(); x++) {				
			Pixel32 p1 = operator()(x, y);

			float newR = p1.r * beta;
			float newG = p1.g * beta;
			float newB = p1.b * beta;

			if (x+1 < width()) {
				auto p1 = operator()(x+1, y);
				newR += p1.r * alpha;
				newG += p1.g * alpha;
				newB += p1.b * alpha;

				if(y+1 < height()) {
					auto p1 = operator()(x+1, y+1);
					newR += p1.r * alpha;
					newG += p1.g * alpha;
					newB += p1.b * alpha;
				}

				if(y-1 > 0) {
					auto p1 = operator()(x+1, y-1);
					newR += p1.r * alpha;
					newG += p1.g * alpha;
					newB += p1.b * alpha;

				}

			}

			if (x-1 > 0) {
				auto p1 = operator()(x-1, y);
				newR += p1.r * alpha;
				newG += p1.g * alpha;
				newB += p1.b * alpha;

				if(y+1 < height()) {
					newR += p1.r * alpha;
					newG += p1.g * alpha;
					newB += p1.b * alpha;
				}

				if(y-1 > 0) {
					newR += p1.r * alpha;
					newG += p1.g * alpha;
					newB += p1.b * alpha;
				}
			
			}

			if(y+1 < height()) {
				auto p1 = operator()(x, y+1);
				newR += p1.r * alpha;
				newG += p1.g * alpha;
				newB += p1.b * alpha;
			}

			if(y-1 > 0) {
				auto p1 = operator()(x, y-1);
				newR += p1.r * alpha;
				newG += p1.g * alpha;
				newB += p1.b * alpha;
			}

			p1.r = clampRGB(newR);
			p1.g = clampRGB(newG);
			p1.b = clampRGB(newB);

			outputImage(x, y) = p1;
		}
	}

	return outputImage;
}

Image32 Image32::scaleNearest( double scaleFactor ) const
{

	/////////////////////////////////////////////////
	// Do scaling with nearest-point sampling here //
	/////////////////////////////////////////////////

	Image32 outputImage = Image32();
	outputImage.setSize(width()*scaleFactor, height()*scaleFactor);

	int height = outputImage.height();
	int width = outputImage.width();

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {				
			Point2D p((double)x/scaleFactor, (double)y/scaleFactor);

			Pixel32 p1 = nearestSample(p);
			outputImage(x, y) = p1;
			
		}
	}

	return outputImage;
}

Image32 Image32::scaleBilinear( double scaleFactor ) const
{
	////////////////////////////////////////////
	// Do scaling with bilinear sampling here //
	////////////////////////////////////////////
	Image32 outputImage = Image32();
	outputImage.setSize(width()*scaleFactor, height()*scaleFactor);

	int height = outputImage.height();
	int width = outputImage.width();

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {				
			Point2D p((double)x/scaleFactor, (double)y/scaleFactor);

			Pixel32 p1 = bilinearSample(p);

			p1.r = clampRGB(p1.r);
			p1.g = clampRGB(p1.g);
			p1.b = clampRGB(p1.b);

			outputImage(x, y) = p1;
		}
	}

	return outputImage;
}

Image32 Image32::scaleGaussian( double scaleFactor ) const
{
	////////////////////////////////////////////
	// Do scaling with Gaussian sampling here //
	////////////////////////////////////////////

	Image32 outputImage = Image32();
	outputImage.setSize(width()*scaleFactor, height()*scaleFactor);

	int height = outputImage.height();
	int width = outputImage.width();

	double radius = 2/scaleFactor;
	double variance = pow(radius/3, 2);

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {				
			Point2D p((double)x/scaleFactor, (double)y/scaleFactor);

			Pixel32 p1 = gaussianSample(p, variance, radius);

			p1.r = clampRGB(p1.r);
			p1.g = clampRGB(p1.g);
			p1.b = clampRGB(p1.b);

			outputImage(x, y) = p1;
		}
	}

	return outputImage;
}

Image32 Image32::rotateNearest( double angle ) const
{
	//////////////////////////////////////////////////
	// Do rotation with nearest-point sampling here //
	//////////////////////////////////////////////////

	Image32 outputImage = Image32();

	double radians = (angle * M_PI) / 180;
	float cosine = cos(radians);
	float sine = sin(radians);

	int height = (this->height()*cosine) + (this->width()*sine);
	int width = (this->width()*cosine) + (this->height()*sine);

	outputImage.setSize(height, width);
	
	int x0 = 0.5 * (this->width()); // point to rotate about
    int y0 = 0.5 * (this->height()); // center of image

	for (int y = 0; y < width; y++) {
		for (int x = 0; x < height; x++) {	
			long double a = x - (width/2);
            long double b = y - (height/2);
            double xx = (+a * cosine - b * sine) + x0;
            double yy = (+a * sine + b * cosine) + y0;
			Point2D p((double)xx, (double)yy);

			Pixel32 p1 = nearestSample(p);

			outputImage(x, y) = p1; 
		}
	}

	return outputImage;
}

Image32 Image32::rotateBilinear( double angle ) const
{
	/////////////////////////////////////////////
	// Do rotation with bilinear sampling here //
	/////////////////////////////////////////////
	Image32 outputImage = Image32();

	double radians = (angle * M_PI) / 180;
	float cosine = cos(radians);
	float sine = sin(radians);

	int height = (this->height()*cosine) + (this->width()*sine);
	int width = (this->width()*cosine) + (this->height()*sine);

	outputImage.setSize(height, width);
	
	int x0 = 0.5 * (this->width()); // point to rotate about
    int y0 = 0.5 * (this->height()); // center of image

	for (int y = 0; y < width; y++) {
		for (int x = 0; x < height; x++) {	
			long double a = x - (width/2);
            long double b = y - (height/2);
            double xx = (+a * cosine - b * sine) + x0;
            double yy = (+a * sine + b * cosine) + y0;
			Point2D p((double)xx, (double)yy);

			Pixel32 p1 = bilinearSample(p);

			outputImage(x, y) = p1; 
		}
	}

	return outputImage;
}

Image32 Image32::rotateGaussian( double angle ) const
{
	/////////////////////////////////////////////
	// Do rotation with Gaussian sampling here //
	/////////////////////////////////////////////
	Image32 outputImage = Image32();

	double radians = (angle * M_PI) / 180;
	float cosine = cos(radians);
	float sine = sin(radians);

	int height = (this->height()*cosine) + (this->width()*sine);
	int width = (this->width()*cosine) + (this->height()*sine);

	double radius = 2/(height/this->height());
	double variance = pow(radius/3, 2);

	outputImage.setSize(height, width);
	
	int x0 = 0.5 * (this->width()); // point to rotate about
    int y0 = 0.5 * (this->height()); // center of image

	for (int y = 0; y < width; y++) {
		for (int x = 0; x < height; x++) {	
			long double a = x - (width/2);
            long double b = y - (height/2);
            double xx = (+a * cosine - b * sine) + x0;
            double yy = (+a * sine + b * cosine) + y0;
			Point2D p((double)xx, (double)yy);

			Pixel32 p1 = gaussianSample(p, variance, radius);

			outputImage(x, y) = p1; 
		}
	}

	return outputImage;
}

void Image32::setAlpha( const Image32& matte )
{
	///////////////////////////
	// Set alpha values here //
	///////////////////////////
	WARN( "method undefined" );
}

Image32 Image32::composite( const Image32& overlay ) const
{
	/////////////////////////
	// Do compositing here //
	/////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::CrossDissolve( const Image32& source , const Image32& destination , double blendWeight )
{
	////////////////////////////
	// Do cross-dissolve here //
	////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::warp( const OrientedLineSegmentPairs& olsp ) const
{
	/////////////////////
	// Do warping here //
	/////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::funFilter( void ) const
{
	////////////////////////////
	// Do the fun-filter here //
	////////////////////////////

	Image32 outputImage = Image32();
	outputImage.setSize(width(), height());

	float swirlX = width()/2;
	float swirlY = height()/2;
	float swirlRadius = width()/2; //arbitrary 
	float swirlTwists = 2;

	for(int y = 0; y < height(); y++) {
    	for(int x = 0; x < width(); x++) {
			// compute the distance and angle from the swirl center:
			float pixelX = (float)x - swirlX;
			float pixelY = (float)y - swirlY;
			float pixelDistance = sqrt(pow(pixelX, 2) + pow(pixelY, 2));
			float pixelAngle = atan2(pixelY, pixelX);

			// work out how much of a swirl to apply (1.0 in the center fading out to 0.0 at the radius):
			float swirlAmount = 1.0f - (pixelDistance / swirlRadius);
			if(swirlAmount > 0.0f) {
				float twistAngle = swirlTwists * swirlAmount * M_PI * 2.0;

				// adjust the pixel angle and compute the adjusted pixel co-ordinates:
				pixelAngle += twistAngle;
				pixelX = cos(pixelAngle) * pixelDistance;
				pixelY = sin(pixelAngle) * pixelDistance;
			}

			// read and write the pixel
			outputImage(x, y) = this->operator()(swirlX + pixelX, swirlY + pixelY);
    	}
	}
	
	return outputImage;
}

Image32 Image32::crop( int x1 , int y1 , int x2 , int y2 ) const
{
	//////////////////////
	// Do cropping here //
	//////////////////////

	Image32 outputImage = Image32();

	int height = y2-y1;
	int width = x2-x1;

	outputImage.setSize(height, width);
	
	for (int y = 0; y < width; y++) {
		for (int x = 0; x < height; x++) {	
			Pixel32 p1 = this->operator()(x1+x, y1+y);
			outputImage(x,y) = p1;
		}
	}

	return outputImage;
}

Pixel32 Image32::nearestSample( Point2D p ) const
{
	//////////////////////////////
	// Do nearest sampling here //
	//////////////////////////////

	int tempX = floor(p[0] + 0.5);
	int tempY = floor(p[1] + 0.5);

	if(tempX >= 0 && tempX < this->width() && tempY >=0 && tempY < this->height()){
		return this->operator()(tempX, tempY);
	}

	return Pixel32();
}

Pixel32 Image32::bilinearSample( Point2D p ) const
{
	///////////////////////////////
	// Do bilinear sampling here //
	///////////////////////////////

	double x = p[0];
	double y = p[1];
	double x1 = floor(p[0]); 
	double y1 = floor(p[1]);
	double x2 = x1 + 1;
	double y2 = y1 + 1;
	double dx = x - x1;

	Pixel32 p1a;
	if(x1 >= 0 && x1 < this->width() && y1 >= 0 && y1 < this->height()) {
		p1a = this->operator()(x1, y1);
	} else {
		p1a = Pixel32();
	}

	Pixel32 p2a;
	if(x2 >= 0 && x2 < this->width() && y1 >= 0 && y1 < this->height()) {
		p2a = this->operator()(x2, y1);
	} else {
		p2a = Pixel32();
	}

	Pixel32 pA;
	pA.r = (p1a.r * (1-dx)) + (p2a.r * (dx));
	pA.g = (p1a.g * (1-dx)) + (p2a.g * (dx));
	pA.b = (p1a.b * (1-dx)) + (p2a.b * (dx));

	Pixel32 p1b;
	if(x1 >= 0 && x1 < this->width() && y2 >= 0 && y2 < this->height()) {
		p1b = this->operator()(x1, y2);
	} else {
		p1b = Pixel32();
	}

	Pixel32 p2b;
	if(x2 >= 0 && x2 < this->width() && y2 >= 0 && y2 < this->height()) {
		p2b = this->operator()(x2, y2);
	} else {
		p2b = Pixel32();
	}

	Pixel32 pB;
	pB.r = (p1b.r * (1-dx)) + (p2b.r * (dx));
	pB.g = (p1b.g * (1-dx)) + (p2b.g * (dx));
	pB.b = (p1b.b * (1-dx)) + (p2b.b * (dx));

	double dy = y - y1;

	Pixel32 temp;
	temp.r = (pA.r * (1-dy)) + (pB.r * dy);
	temp.g = (pA.g * (1-dy)) + (pB.g * dy);
	temp.b = (pA.b * (1-dy)) + (pB.b * dy);

	return temp;
}

Pixel32 Image32::gaussianSample( Point2D p , double variance , double radius ) const
{
	///////////////////////////////
	// Do Gaussian sampling here //
	///////////////////////////////
	double x0 = p[0];
	double y0 = p[1];

	Pixel32 pix;

	for (int y = fmax(0, floor(y0 - radius)); y <= fmin(ceil(y0 + radius), height()-1); y++) {
		for (int x = fmax(0, floor(x0 - radius)); x <= fmin(ceil(x0 + radius), width()-1); x++) {
			double dist = (pow(x-x0, 2) + pow(y-y0, 2));
			double gauss = (1/(2* (M_PI) * variance)) * exp((-1/(2*variance)) * dist);
			
			pix.r += operator()(x,y).r * (gauss);
			pix.g += operator()(x,y).g * (gauss);
			pix.b += operator()(x,y).b * (gauss);
		}
	}

	pix.r = pix.r;
	pix.g = pix.g; 
	pix.b = pix.b; 

	return pix;
}
