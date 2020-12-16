// Image processing classes for beacon detector
// Authors: Ondrej Cierny, Peter Grenfell
#include "processing.h"
#include <cmath>
#include <functional>
#include <algorithm>
#include <memory>

//-----------------------------------------------------------------------------
Image::Image(Camera &camera, std::ofstream &fileStreamIn, zmq::socket_t &pat_health_port_in, int smoothing): 
fileStream(fileStreamIn), pat_health_port(pat_health_port_in)
//-----------------------------------------------------------------------------
{
	// Copy properties
	const Request *request = camera.getRequest();
	binningMode = camera.config->binningMode.read();
	size = request->imageSize.read();
	area.w = request->imageWidth.read();
	area.h = request->imageHeight.read();
	area.x = request->imageOffsetX.read();
	area.y = request->imageOffsetY.read();
	data = new uint16_t[area.w*area.h];
	memcpy(data, request->imageData.read(), size);

	// Unlock camera request
	camera.unlockRequest();

	// Smoothing
	applyFastBlur(smoothing);

	// Basic histogram analysis, find peak and brightest pixel
	int histogram[1024], maxCount = 0, sum = 0;
	memset(histogram, 0, 1024*sizeof(int));
	histBrightest = 0;
	for(int i = 0; i < area.w*area.h; i++)
	{
		if(data[i] > 1023)
		{
			log(pat_health_port, fileStream, "In processing.cpp Image::Image - Brightness overflow detected in frame");
			continue;
		}
		sum += data[i];
		histogram[data[i]]++;
		if(histogram[data[i]] > maxCount)
		{
			maxCount = histogram[data[i]];
			histPeak = data[i];
		}
		if(data[i] > histBrightest) histBrightest = data[i];
	}
	histMean = sum / (area.w * area.h);
}

//-----------------------------------------------------------------------------
Image::~Image()
//-----------------------------------------------------------------------------
{
	delete data;
}

// Maps neighboring pixels to groups and performs group centroiding
// TODO: seems to crash sometimes when camera is oversaturated, memory problem?!
//-----------------------------------------------------------------------------
int Image::performPixelGrouping(uint16_t threshold)
//-----------------------------------------------------------------------------
{
	unsigned int currentGroup = 0;
	set<int> groupedPixels;
	groups.clear();

	// Determine thresholding if not given
	// if(threshold == 0) threshold = autoThresholdPeakToMax();
	if(threshold == 0) threshold = histMean + THRESHOLD_SAFETY_OFFSET;

	// Helper recursive group-propagation function
	function<void(int,int)> propagateGroup;
	propagateGroup = [this, threshold, &currentGroup, &groupedPixels, &propagateGroup](int i, int j)
	{
		// Acknowledge parent pixel in group
		int index = i*area.w + j;
		Group& g = groups[currentGroup];
		g.pixelCount++;
		g.valueSum += data[index];
		g.x += j * data[index];
		g.y += i * data[index];
		if(data[index] > g.valueMax) g.valueMax = data[index];
		groupedPixels.insert(index);

		// Fail-safe
		if(groupedPixels.size() > MAX_ACTIVE_PIXELS) return;

		// Loop through neighbors
		for(int8_t x = -1; x <= 1; x++)
		{
			for(int8_t y = -1; y <= 1; y++)
			{
				if((i+x) >= 0 && (i+x) < area.h && (j+y) >= 0 && (j+y) < area.w)
				{
					index = (i+x)*area.w + j + y;
					if(groupedPixels.count(index) == 0 && data[index] > threshold)
					{
						propagateGroup(i+x, j+y);
					}
				}
			}
		}
	};

	// Scan all pixels
	for(int i = 0; i < area.h; i++)
	{
		for(int j = 0; j < area.w; j++)
		{
			int index = i*area.w + j;
			if(data[index] > threshold && groupedPixels.count(index) == 0)
			{
				// Ungrouped pixel found, propagate a new group
				groups.emplace_back();
				propagateGroup(i, j);

				// Verify number of pixels in group after propagation
				if(groups[currentGroup].pixelCount < MIN_PIXELS_PER_GROUP)
				{
					groups.pop_back();
					continue;
				}

				// Too many active pixels, must be background, clear and return
				if(groupedPixels.size() > MAX_ACTIVE_PIXELS)
				{
					log(pat_health_port, fileStream, "In processing.cpp Image::performPixelGrouping - Frame has too many active pixels!");
					groups.clear();
					return -1;
				}

				// Calculate centroid for group
				groups[currentGroup].x /= groups[currentGroup].valueSum;
				groups[currentGroup].y /= groups[currentGroup].valueSum;

				// Increase and check group count
				if(++currentGroup > MAX_GROUPS)
				{
					log(pat_health_port, fileStream, "In processing.cpp Image::performPixelGrouping - Frame has too many groups!");
					sort(groups.begin(), groups.end());
					return -1;
				}
			}
		}
	}

	// Sort max-brightness-descending
	if(groups.size() > 0) sort(groups.begin(), groups.end());
	else log(pat_health_port, fileStream, "In processing.cpp Image::performPixelGrouping - Frame has no groups!");

	return groups.size();
}

// Super-fast box blur algorithm, converges to Gaussian with more passes
// Adapted from: http://blog.ivank.net/fastest-gaussian-blur.html
//-----------------------------------------------------------------------------
void Image::applyFastBlur(double radius, double passes)
//-----------------------------------------------------------------------------
{
	if(radius < 1) return;
	uint16_t *temp = new uint16_t[area.w*area.h];
	memcpy(temp, data, size);

	// Calculate boxes area.w
	int wl = sqrt((12*radius*radius/passes)+1);
	if(wl % 2 == 0) wl--;
	int m = ((12*radius*radius - passes*wl*wl - 4*passes*wl - 3*passes)/(-4*wl - 4)) + 0.5f;

	// Blur for n passes
	for(int8_t n = 0; n < passes; n++)
	{
		int r = ((n < m ? wl : wl + 2) - 1) / 2;
		float iarr = 1.0f / (r+r+1);

		// Apply horizontal blur
		for(int i = 0; i < area.h; i++)
		{
			int ti = i*area.w, li = ti, ri = ti + r;
			int fv = data[ti], lv = data[ti+area.w-1], val = (r+1)*fv;
			for(int j = 0; j < r; j++) val += data[ti+j];
			for(int j = 0; j <= r; j++) { val += data[ri++] - fv; temp[ti++] = val*iarr + 0.5f; }
			for(int j = r+1; j < area.w-r; j++) { val += data[ri++] - data[li++]; temp[ti++] = val*iarr + 0.5f; }
			for(int j = area.w-r; j < area.w; j++) { val += lv - data[li++]; temp[ti++] = val*iarr + 0.5f; }
		}
		memcpy(data, temp, size);

		// Apply total blur
		for(int i = 0; i < area.w; i++)
		{
			int ti = i, li = ti, ri = ti + r*area.w;
			int fv = data[ti], lv = data[ti + area.w*(area.h-1)], val = (r+1)*fv;
			for(int j = 0; j < r; j++) val += data[ti + j*area.w];
			for(int j = 0; j <= r; j++) { val += data[ri] - fv; temp[ti] = val*iarr + 0.5f; ri += area.w; ti += area.w; }
			for(int j = r+1; j < area.h-r; j++) { val += data[ri] - data[li]; temp[ti] = val*iarr + 0.5f; li += area.w; ri += area.w; ti += area.w; }
			for(int j = area.h-r; j < area.h; j++) { val += lv - data[li]; temp[ti] = val*iarr + 0.5f; li += area.w; ti += area.w; }
		}
		memcpy(data, temp, size);
	}

	delete temp;
}

// Threshold as a fraction of distance between histogram peak and max brightness
//-----------------------------------------------------------------------------
uint16_t Image::autoThresholdPeakToMax(float fraction)
//-----------------------------------------------------------------------------
{
	int treshold = 1023;
	if(histBrightest > histPeak)
	{
		treshold = histPeak + ((histBrightest - histPeak) / fraction);
	}
	return treshold;
}

//Image Saving Capability: save to BMP, adapted from: https://www.matrix-vision.com/manuals/SDK_CPP/SingleCaptureStorage_8cpp-example.html
//-----------------------------------------------------------------------------
void Image::saveBMP(const string& filename) //e.g. const string filename( "single.bmp" );
//const char* pData, int XSize, int YSize, int pitch, int bitsPerPixel
//------------------------------------------------------------------------------
{
	int newHeight = area.h, newWidth = area.w, ratio = 1;
	shared_ptr<string> ImgData = make_shared<string>();

	// Determine if resizing is needed
	if(area.h > LINK_MAX_FRAME_HEIGHT)
	{
		ratio = area.h / LINK_MAX_FRAME_HEIGHT + 1;
		if(ratio == 3 || ratio > 4) ratio = 4;
		newHeight = area.h / ratio;
		newWidth = area.w / ratio;
	}

	// Resize, downsample to 8-bit, flip vertically (ref. BMP format)
	switch(ratio)
	{
		case 1:
			for(int i = newHeight - 1; i >= 0; i--)
			{
				for(int j = 0; j < newWidth; j++)
				{
					ImgData->append(1, data[(i*area.w)+j] >> 2);
				}
			}
			break;
		case 2:
			for(int i = newHeight - 1; i >= 0; i--)
			{
				int ii0 = i * area.w * 2;
				int ii1 = ii0 + area.w;
				for(int j = 0; j < newWidth; j++)
				{
					int jj0 = 2 * j;
					int jj1 = jj0 + 1;
					ImgData->append(1, (data[ii0 + jj0] + data[ii0 + jj1] +
									 data[ii1 + jj0] + data[ii1 + jj1]) >> 4);
				}
			}
			break;
		default:
			for(int i = newHeight - 1; i >= 0; i--)
			{
				int ii0 = i * area.w * 4;
				int ii1 = ii0 + area.w;
				int ii2 = ii1 + area.w;
				int ii3 = ii2 + area.w;
				for(int j = 0; j < newWidth; j++)
				{
					int jj0 = 4 * j;
					int jj1 = jj0 + 1;
					int jj2 = jj1 + 1;
					int jj3 = jj2 + 1;
					ImgData->append(1, (data[ii0 + jj0] + data[ii0 + jj1] +
									 data[ii0 + jj2] + data[ii0 + jj3] +
									 data[ii1 + jj0] + data[ii1 + jj1] +
									 data[ii1 + jj2] + data[ii1 + jj3] +
									 data[ii2 + jj0] + data[ii2 + jj1] +
									 data[ii1 + jj2] + data[ii2 + jj3] +
									 data[ii3 + jj0] + data[ii3 + jj1] +
									 data[ii1 + jj2] + data[ii3 + jj3]) >> 6);
				}
			}
	}

	try{
		BMP AnImage;
		AnImage.SetSize(newWidth,newHeight);
		// Set its color depth to 8-bits
		AnImage.SetBitDepth(8);
		log(pat_health_port, fileStream, "In processing.cpp Image::saveBMP - Image Set Up, Beginning Saving...");
		int ImgDataSize = ImgData->length();
		log(pat_health_port, fileStream, "Size: ", ImgDataSize, ", Width: ", newWidth, ", Height: ", newHeight);
		for (int j = 0; j < newHeight; j++)
			{
				for (int i = 0; i < newWidth; i++)
					{
						 // Set one of the pixels
						 unsigned int temporary = (unsigned int)(ImgData->at(j*newWidth + i));
						 AnImage(i,j)->Red = temporary;
						 AnImage(i,j)->Green = temporary;
						 AnImage(i,j)->Blue = temporary;
						 AnImage(i,j)->Alpha = 0; //always set to zero;
					}
			}
		CreateGrayscaleColorTable(AnImage); //we're using 8-bit greyscale
		AnImage.WriteToFile(filename.c_str());
	}
	catch(const std::exception& e){
		log(pat_health_port, fileStream, e.what()); // information from length_error printed
	}
	catch(...){
		log(pat_health_port, fileStream, "In processing.cpp Image::saveBMP - Error during image saving using EasyBMP functions.");
	}
}

// Image Saving Capability: save to PNG - adapted from github.mit.edu: CLICK_TVAC_Support/Dark/main.cpp (Author: Ondrej Cierny)
//-----------------------------------------------------------------------------
void Image::savePNG(std::string fileName)
//-----------------------------------------------------------------------------
{
	vector<unsigned char> png;
	png.resize(area.w * area.h * 2);

	// Copy 10 bit data into 16 bit format
	for (unsigned y = 0; y < area.h; y++)
	{
		for (unsigned x = 0; x < area.w; x++)
		{
			png[2 * area.w * y + 2 * x + 0] = data[(y*area.w) + x] >> 2; // most significant
			png[2 * area.w * y + 2 * x + 1] = data[(y*area.w) + x] << 6; // least significant
		}
	}

	// Notify main thread we finished copying data
	// c.notify_one(); //original used multi-threading to minimize delays (will try without this first for code stability).

	// Begin encoding into 16-bit greyscale PNG
	unsigned error = lodepng::encode(fileName, png, area.w, area.h, LCT_GREY, 16);

	if (error != 0)
	{
		log(pat_health_port, fileStream, "In processing.cpp Image::savePNG - Encoding error:", error);
	}
}

//-----------------------------------------------------------------------------
// Log Image from Camera
//-----------------------------------------------------------------------------
void logImage(std::string nameTag, Camera& cameraObj, std::ofstream& textFileIn, zmq::socket_t& pat_health_port, bool save_extra_exposures, std::string fileType)
{
	int exposure_init = cameraObj.config->expose_us.read(); //get current camera exposure	
	int exposures[3];
	exposures[0] = exposure_init;
	int num_images = 1;
	if(save_extra_exposures){
		exposures[1] = (exposure_init + MIN_EXPOSURE)/2; num_images++;
		exposures[2] = (exposure_init + MAX_EXPOSURE)/2; num_images++;
		if(exposures[1] < MIN_EXPOSURE){exposures[1] = MIN_EXPOSURE;}
		if(exposures[1] > MAX_EXPOSURE){exposures[1] = MAX_EXPOSURE;}
		if(exposures[2] < MIN_EXPOSURE){exposures[2] = MIN_EXPOSURE;}
		if(exposures[2] > MAX_EXPOSURE){exposures[2] = MAX_EXPOSURE;}
	}	
	
	for(int i = 0; i < num_images; i++){
		cameraObj.config->expose_us.write(exposures[i]); //set camera exposure
		cameraObj.requestFrame(); //queue frame
		if(cameraObj.waitForFrame())
		{
			Image frame(cameraObj, textFileIn, pat_health_port);
			
			if((fileType.compare(std::string("bmp")) == 0) || (fileType.compare(std::string("BMP")) == 0)){
				const std::string imageFileName = timeStamp() + std::string("_") + nameTag + std::string("_exp_") + std::to_string(exposures[i]) + std::string(".bmp");
				log(pat_health_port, textFileIn, "In processing.cpp logImage - Saving image telemetry as: ", imageFileName);
				frame.saveBMP(imageFileName);
			} else if((fileType.compare(std::string("png")) == 0) || (fileType.compare(std::string("PNG")) == 0)){
				std::string imageFileName = timeStamp() + std::string("_") + nameTag + std::string("_exp_") + std::to_string(exposures[i]) + std::string(".png");
				log(pat_health_port, textFileIn, "In processing.cpp logImage - Saving image telemetry as: ", imageFileName);
				frame.savePNG(imageFileName);
			} else{
				log(pat_health_port, textFileIn, "In processing.cpp logImage - Error: unrecognized file type ", fileType);
			}
		}
		else
		{
			log(pat_health_port, textFileIn, "In processing.cpp logImage - Error: waitForFrame");
		}
	}
	
	if(save_extra_exposures){
		cameraObj.config->expose_us.write(exposure_init); //reset camera exposure
	}
}
