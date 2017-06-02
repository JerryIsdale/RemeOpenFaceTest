#include <stdint.h>
#include <string>
#include <reconstructmesdk/reme.h>

#include "MetaReme.h"

using std::string;

#pragma region Create references to reme variables
reme_context_t scanContext;
reme_sensor_t sensor;

reme_image_t colorImage, volumeImage;

reme_volume_t scanVolume;
reme_surface_t scanMesh;
#pragma endregion

#pragma region Define global boolean flags to be used
static bool sensorIsInitialized = false;
static bool applicationIsStreaming = false;
static bool applicationIsScanning = false;
static bool shouldResetVolume = false;
#pragma endregion

#pragma region Define global callback functions to be used
static void(*colorImageCallback)(uint8_t *bytes, int size, int w, int h) = NULL;
static void(*volumeImageCallback)(uint8_t *bytes, int size, int w, int h) = NULL;

static void(*errorLoggingCallback)(const char *error) = NULL;

static void(*stopStreamingCallback)(int status) = NULL;
#pragma endregion

#pragma region Define external API functions
/**
* \brief Begin a new ReconstructMe session.
*
* This will assign the color and volume rendering callbacks, set options,
* and create the ReconstructMe context.
*/
bool beginSession(
	void(*colorCallback)(uint8_t *bytes, int size, int w, int h),
	void(*volumeCallback)(uint8_t *bytes, int size, int w, int h)
)
{
	// Assign the streaming callback functions
	colorImageCallback = colorCallback;
	volumeImageCallback = volumeCallback;

	// Assign the reconstructme context to the global variable
	checkForErr(reme_context_create(&scanContext), "Create context");

	// Set up the hard coded reconstruction options to use
	setReconstructionOptions();

	// Compile the reconstructme contex for OpenCL
	checkForErr(reme_context_compile(scanContext), "Compile context");

	return true;
}

/**
* \brief End the ReconstructMe session.
*
* This will tear down the existing ReconstructMe resources (volume, mesh, and context).
* If the API is currently streaming, stopStreaming should be closed and resolved before
* endSession is called.
*/
bool endSession()
{
	// Destroy Volume
	checkForErr(reme_volume_destroy(scanContext, &scanVolume), "Cleanup - Destroy Volume");

	// Destroy Mesh
	checkForErr(reme_surface_destroy(scanContext, &scanMesh), "Cleanup - Destroy Surface");

	// Destroy context
	checkForErr(reme_context_destroy(&scanContext), "Cleanup - Destroy Context");

	return true;
}

/**
* \brief Set the callback to be called when errors occur
*
* The callback is a function that will be called with a const char* parameter
*/
void setErrorCallback(void(*callback)(const char *error))
{
	errorLoggingCallback = callback;
}

/**
* \brief Start streaming frames to the render callback functions
*
* This will start the sensor (camera) and start the render loop, which by default streams
* color frames.
* WARNING: This function must be called as an async function. Failing to do this will block the
* code in an infinite while loop
*/
void startStreaming()
{
	// Start the sensor with hardcoded camera options
	startSensor();

	//  Prep for streaming
	applicationIsStreaming = true;
	checkForErr(reme_image_create(scanContext, &colorImage), "Create Color Image");

	// Run the render loop
	runRenderLoop();
}

/**
* \brief Stop streaming frames to the render callback functions
*
* This will stop the sensor and break out of the render loop. The status callback will
* be called when the render loop has successfully finished and cleaned up.
*/
void stopStreaming(void(*statusCallback)(int status))
{
	// Stop Sensor
	stopSensor();

	// Set the status callback
	stopStreamingCallback = statusCallback;

	// Set the flags to kill the render loop
	applicationIsStreaming = false;
	applicationIsScanning = false;
	shouldResetVolume = false;
}

/**
* \brief Start scanning functionality
*
* This creates the volume image and scan volume resources.
*/
void startScan()
{
	// Create the volume image
	checkForErr(reme_image_create(scanContext, &volumeImage), "Create Volume Image");
	// Create the scan volume
	checkForErr(reme_volume_create(scanContext, &scanVolume), "Create Scan Volume");

	applicationIsScanning = true;
}

/**
* \brief Reset the scan
*
* This will occur on the next iteration of the render loop
*/
void resetScan()
{
	shouldResetVolume = true;
}

/**
* \brief Generate the mesh from the captured scan data
*
* The application should have already collected scan data by running startScan while
* the application is streaming. This will generate the surface from the collected data.
*/
void generateMesh(char *captureFilename, void(*statusCallback)(int status))
{
	// Create the surface mesh
	checkForErr(reme_surface_create(scanContext, &scanMesh), "Create Surface Mesh");

	// Create the surface options
	reme_options_t surfaceOptions;
	checkForErr(reme_options_create(scanContext, &surfaceOptions), "Create Surface Options");

	// Set the generation to use the specified options
	checkForErr(reme_surface_bind_generation_options(scanContext, scanMesh, surfaceOptions), "Bind Options");
	checkForErr(
		reme_options_set_bool(scanContext, surfaceOptions, "merge_duplicate_vertices", true),
		"Set Merge Option"
	);

	// Generate the surface
	checkForErr(reme_surface_generate(scanContext, scanMesh, scanVolume), "Generate Surface");

	// Save to the specified file
	//checkForErr(reme_surface_save_to_file(scanContext, scanMesh, captureFilename), "Save Surface to File");

	// Clean up volume and surface
	//checkForErr(reme_volume_destroy(scanContext, &scanVolume), "Generate - Destroy Volume");
	//checkForErr(reme_surface_destroy(scanContext, &scanMesh), "Generate - Destroy Surface");

	statusCallback(0);
}

void saveMeshToFile(char *captureFilename, void(*statusCallback)(int status))
{
	// Save to the specified file
	checkForErr(reme_surface_save_to_file(scanContext, scanMesh, captureFilename), "Save Surface to File");
	statusCallback(0);
}

void releaseVolumeSurface()
{
	// Clean up volume and surface
	checkForErr(reme_volume_destroy(scanContext, &scanVolume), "Generate - Destroy Volume");
	checkForErr(reme_surface_destroy(scanContext, &scanMesh), "Generate - Destroy Surface");

}

void getSurfacePoints(const float **coordinates, int *length)
{
	checkForErr(reme_surface_get_points(scanContext, scanMesh, coordinates, length), "Get Surface Points");
}

void getSurfaceNormals(const float **coordinates, int *length)
{
	checkForErr(reme_surface_get_normals(scanContext, scanMesh, coordinates, length), "Get Surface Normals");
}

void getSurfaceVertexColors(const float **coordinates, int *length)
{
	checkForErr(reme_surface_get_vertex_colors(scanContext, scanMesh, coordinates, length), "Get Vertex Colors");
}

void getSurfaceTriangles(const size_t **indices, int *length)
{
	checkForErr(reme_surface_get_triangles(scanContext, scanMesh, indices, length), "Get Triangles");
}


#pragma endregion

#pragma region Define private utility functions
void runRenderLoop()
{
	bool isFrameTracked;

	while (applicationIsStreaming) {
		if (shouldResetVolume) {
			// Reset the volume and the sensor
			checkForErr(reme_volume_reset(scanContext, scanVolume), "Reset Volume");
			checkForErr(reme_sensor_reset(scanContext, sensor), "Reset Sensor");

			checkForErr(
				reme_sensor_set_prescan_position(scanContext, sensor, REME_SENSOR_POSITION_INFRONT),
				"Reset - Set Prescan Position"
			);

			shouldResetVolume = false;
			continue;
		}

		if (getCurrentFrame()) {
			if (applicationIsScanning) {
				// Prepare color and depth images
				checkForErr(reme_sensor_prepare_images(scanContext, sensor), "Scanning - Prepare Images");

				// Determine the current sensor position
				isFrameTracked = REME_SUCCESS(reme_sensor_track_position(scanContext, sensor));
				// Update the volume if sensor position is determined
				if (isFrameTracked) reme_sensor_update_volume(scanContext, sensor);

				sendVolumeImage();
			}
			else {
				// Update internal state of images
				checkForErr(
					reme_sensor_prepare_image(scanContext, sensor, REME_IMAGE_AUX),
					"Streaming - Prepare Image"
				);
			}

			// Always send a color frame
			sendColorImage();
		}
	}

	// Teardown the image references
	checkForErr(reme_image_destroy(scanContext, &colorImage), "Stop Streaming - Destroy Color Image");
	checkForErr(reme_image_destroy(scanContext, &volumeImage), "Stop Streaming - Destroy Volume Image");

	if (stopStreamingCallback) stopStreamingCallback(0);
}

void startSensor()
{
	if (sensorIsInitialized) return;

	checkForErr(reme_sensor_create(scanContext, "librealsense", true, &sensor), "Create Sensor");

	reme_options_t captureOptions;
	checkForErr(reme_options_create(scanContext, &captureOptions), "Sensor - Create Options");
	checkForErr(reme_sensor_bind_capture_options(scanContext, sensor, captureOptions), "Sensor - Bind Options");

	checkForErr(
		reme_options_set_int(scanContext, captureOptions, "frame_info.aux_size.width", 1920),
		"Sensor - Set Color Width"
	);
	checkForErr(
		reme_options_set_int(scanContext, captureOptions, "frame_info.aux_size.height", 1080),
		"Sensor - Set Color Height"
	);

	checkForErr(
		reme_options_set_int(scanContext, captureOptions, "frame_info.depth_size.width", 640),
		"Sensor - Set Volume Width"
	);
	checkForErr(
		reme_options_set_int(scanContext, captureOptions, "frame_info.depth_size.height", 480),
		"Sensor - Set Volume Height"
	);

	checkForErr(reme_sensor_open(scanContext, sensor), "Sensor - Open");

	checkForErr(
		reme_sensor_set_prescan_position(scanContext, sensor, REME_SENSOR_POSITION_INFRONT),
		"Sensor - Set Prescan Position"
	);

	sensorIsInitialized = true;
}

void stopSensor()
{
	if (!sensorIsInitialized) return;

	checkForErr(reme_sensor_close(scanContext, sensor), "Sensor - Close");
	checkForErr(reme_sensor_destroy(scanContext, &sensor), "Sensor - Destroy");

	sensorIsInitialized = false;
}

void setReconstructionOptions()
{
	reme_options_t reconstructionOptions;
	reme_options_create(scanContext, &reconstructionOptions);

	reme_context_bind_reconstruction_options(scanContext, reconstructionOptions);

	// volume options
	reme_options_set_real(scanContext, reconstructionOptions, "volume.minimum_corner.x", -150.0f);
	reme_options_set_real(scanContext, reconstructionOptions, "volume.minimum_corner.y", -150.0f);
	reme_options_set_real(scanContext, reconstructionOptions, "volume.minimum_corner.z", -150.0f);

	reme_options_set_real(scanContext, reconstructionOptions, "volume.maximum_corner.x", 150.0f);
	reme_options_set_real(scanContext, reconstructionOptions, "volume.maximum_corner.y", 150.0f);
	reme_options_set_real(scanContext, reconstructionOptions, "volume.maximum_corner.z", 150.0f);

	reme_options_set_int(scanContext, reconstructionOptions, "volume.resolution.x", 512);
	reme_options_set_int(scanContext, reconstructionOptions, "volume.resolution.y", 256);
	reme_options_set_int(scanContext, reconstructionOptions, "volume.resolution.z", 256);

	// depthmap upsampling settings
	reme_options_set_bool(scanContext, reconstructionOptions, "depthmap_upsampling.enabled", false);
	reme_options_set_int(scanContext, reconstructionOptions, "depthmap_upsampling.half_kernel_size", 2);
	reme_options_set_real(scanContext, reconstructionOptions, "depthmap_upsampling.sigma_pixel", 5.0f);
	reme_options_set_real(scanContext, reconstructionOptions, "depthmap_upsampling.sigma_color", 15.0f);

	// erosion settings
	reme_options_set_bool(scanContext, reconstructionOptions, "depthmap_erosion.enabled", true);
	reme_options_set_int(scanContext, reconstructionOptions, "depthmap_erosion.half_kernel_size", 2);
	reme_options_set_int(scanContext, reconstructionOptions, "depthmap_erosion.minimum_depth_step", 30);

	reme_options_set_bool(scanContext, reconstructionOptions, "colormap_erosion.enabled", true);
	reme_options_set_int(scanContext, reconstructionOptions, "colormap_erosion.minimum_depth_step", 30);
	reme_options_set_int(scanContext, reconstructionOptions, "colormap_erosion.half_kernel_size", 2);

	// depthmap smoothing settings
	reme_options_set_bool(scanContext, reconstructionOptions, "depthmap_smoothing.enabled", true);
	reme_options_set_int(scanContext, reconstructionOptions, "depthmap_smoothing.half_kernel_size", 3);
	reme_options_set_real(scanContext, reconstructionOptions, "depthmap_smoothing.sigma_pixel", 9.0f);
	reme_options_set_real(scanContext, reconstructionOptions, "depthmap_smoothing.sigma_depth", 30.0f);

	// data integration settings
	reme_options_set_real(scanContext, reconstructionOptions, "data_integration.truncation", 10.0f);
	reme_options_set_int(scanContext, reconstructionOptions, "data_integration.maximum_weight", 500);
	reme_options_set_real(scanContext, reconstructionOptions, "data_integration.maximum_depth", 1500.0f);
	reme_options_set_real(scanContext, reconstructionOptions, "data_integration.maximum_angle", 70.0f);
	reme_options_set_bool(scanContext, reconstructionOptions, "data_integration.use_colors", true);
	reme_options_set_real(scanContext, reconstructionOptions, "data_integration.minimum_deletion_distance", 40.0f);

	// data extraction settings
	reme_options_set_real(scanContext, reconstructionOptions, "data_extraction.step_factor", 0.7f);
	reme_options_set_real(scanContext, reconstructionOptions, "data_extraction.gradient_step_length", 0.1f);
	reme_options_set_int(scanContext, reconstructionOptions, "data_extraction.surface_vicinity_steps", 4);

	// camera tracking options
	reme_options_set_int(scanContext, reconstructionOptions, "camera_tracking.local_search.maximum_iterations", 100);
	reme_options_set_int(scanContext, reconstructionOptions, "camera_tracking.local_search.pyramid_levels", 3);
	reme_options_set_real(scanContext, reconstructionOptions, "camera_tracking.local_search.maximum_point_distance", 20.0f);
	reme_options_set_real(scanContext, reconstructionOptions, "camera_tracking.local_search.maximum_normal_angle", 30.0f);
	reme_options_set_real(scanContext, reconstructionOptions, "camera_tracking.local_search.color_weight", 0.0f);
	reme_options_set_bool(scanContext, reconstructionOptions, "camera_tracking.local_search.skip_finest_pyramid_level", false);

	reme_options_set_real(scanContext, reconstructionOptions, "camera_tracking.global_search.maximum_seconds", 0.7f);
	reme_options_set_real(scanContext, reconstructionOptions, "camera_tracking.global_search.minimum_inlier_percentage", 0.4f);
	reme_options_set_int(scanContext, reconstructionOptions, "camera_tracking.global_search.minimum_feature_matches", 100);

	reme_options_set_real(scanContext, reconstructionOptions, "camera_tracking.verification.maximum_sdf_change_on_track", 0.04f);
	reme_options_set_real(scanContext, reconstructionOptions, "camera_tracking.verification.maximum_sdf_change_no_track", 0.04f);

	reme_options_set_bool(scanContext, reconstructionOptions, "camera_tracking.use_sensor_data_padding", true);
	reme_options_set_bool(scanContext, reconstructionOptions, "camera_tracking.use_position_forecast", true);

}

bool getCurrentFrame()
{
	return REME_SUCCESS(reme_sensor_grab(scanContext, sensor));
}

void logError(string status) {
	if (errorLoggingCallback) errorLoggingCallback(status.c_str());
}

void checkForErr(reme_error_t err, string msg)
{
	if (REME_FAILED(err)) {
		switch (err) {
		case REME_ERROR_UNSPECIFIED:
			logError(string("Unspecified error: ") + msg);
			break;
		case REME_ERROR_FAILED_TO_GRAB:
			logError(string("Failed to grab from sensor: ") + msg);
			break;
		case REME_ERROR_TRACK_LOST:
			logError(string("Camera tracking lost: ") + msg);
			break;
		case REME_ERROR_INVALID_LICENSE:
			logError(string("Invalid license: ") + msg);
			break;
		case REME_ERROR_NO_CALIBRATION_TARGET:
			logError(string("No calibration target found in image: ") + msg);
			break;
		case REME_ERROR_NO_COLOR_SUPPORT:
			logError(string("Color support not available. Enable in config: ") + msg);
			break;
		case REME_ERROR_NO_FLOOR:
			logError(string("Floor detection failed: ") + msg);
			break;
		}
	}
}
#pragma endregion

#pragma region Define sending image helpers
// Hardcoded to 640 x 480. in C++ we can't do something like
// uint8_t rgbaImage[width * height * 4); because width and height aren't
// constant variables.
// TODO: Revisit this
static const int width = 640;
static const int height = 480;
static const int imagePixels = width * height;
static const int rgbaImageSize = imagePixels * 4; //! hard coding the image size
static const int minVolPixSum = 160;

static uint8_t rgbaColorImage[rgbaImageSize];
static uint8_t rgbaVolumeImage[rgbaImageSize];

//! get RGB bytes from remImage, map into RGBA array and send to callback
//! isColor = send via Color callback, otherwise use Volume callback
void remapToRGBA(reme_image_t remeImage, uint8_t* rgbaImage, bool isColor) {
	// Extract the bytes, width and height
	const void *rgbImageBytes;
	int length;
	checkForErr(reme_image_get_bytes(scanContext, remeImage, &rgbImageBytes, &length), "Get Image Bytes");

	// Get the info from the image
	int width, height;
	int nChannels, nbChannel, rStride;
	checkForErr(reme_image_get_info(scanContext, remeImage,
		&width, &height, &nChannels, &nbChannel, &rStride), "Get IPmage Info");

	// Create variables
	int rgbIdx, rgbaIdx, pixSum;
	uint8_t rVal, gVal, bVal;

	for (int i = 0; i < imagePixels; i++) {
		rgbIdx = 3 * i;
		rgbaIdx = 4 * i;

		rVal = ((uint8_t*)rgbImageBytes)[rgbIdx];
		gVal = ((uint8_t*)rgbImageBytes)[rgbIdx + 1];
		bVal = ((uint8_t*)rgbImageBytes)[rgbIdx + 2];

		rgbaImage[rgbaIdx] = rVal;
		rgbaImage[rgbaIdx + 1] = gVal;
		rgbaImage[rgbaIdx + 2] = bVal;

		// Set alpha channel to 255 (opaque) or 0 (transparent)
		// If it's a color image always opaque, otherwise check whether the pixel value is below a threshold
		if (isColor) {
			rgbaImage[rgbaIdx + 3] = 255;
		}
		else {
			pixSum = rVal + gVal + bVal;
			rgbaImage[rgbaIdx + 3] = ((pixSum <= minVolPixSum) ? 0 : 255);
		}
	}
}

void sendColorImage()
{
	if (colorImageCallback == NULL) return;

	reme_sensor_get_image(scanContext, sensor, REME_IMAGE_AUX, colorImage);
	remapToRGBA(colorImage, rgbaColorImage, true);

	if (colorImageCallback != NULL) (*colorImageCallback)(rgbaColorImage, rgbaImageSize, width, height);
}

void sendVolumeImage()
{
	if (volumeImageCallback == NULL) return;

	reme_sensor_get_image(scanContext, sensor, REME_IMAGE_VOLUME, volumeImage);
	remapToRGBA(volumeImage, rgbaVolumeImage, false);

	if (volumeImageCallback != NULL) (*volumeImageCallback)(rgbaVolumeImage, rgbaImageSize, width, height);
}
#pragma endregion