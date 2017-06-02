#pragma once

#include <stdint.h>
#include <string>
#include <reconstructmesdk/reme.h>

using std::string;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * The following ifdef block is the standard way of creating macros which make exporting 
 * from a DLL simpler. All files within this DLL are compiled with the METAREME_EXPORTS
 * symbol defined on the command line. This symbol should not be defined on any project
 * that uses this DLL. This way any other project whose source files include this file see
 * METAREME_API functions as being imported from a DLL, whereas this DLL sees symbols
 * defined with this macro as being exported.
*/
#ifdef METAREME_EXPORTS
#define METAREME_API __declspec(dllexport)
#else
#define METAREME_API 
#endif
	METAREME_API bool beginSession(
		void(*colorCallback)(uint8_t *bytes, int size, int w, int h),
		void(*volumeCallback)(uint8_t *bytes, int size, int w, int h)
	);
	METAREME_API bool endSession(void);

	METAREME_API void setErrorCallback(void(*callback)(const char *error));

	METAREME_API void startStreaming(void);
	METAREME_API void stopStreaming(void(*statusCallback)(int status));

	METAREME_API void startScan(void);
	METAREME_API void resetScan(void);

	METAREME_API void generateMesh(char *captureFilename, void(*statusCallback)(int status));
	METAREME_API void saveMeshToFile(char *captureFilename, void(*statusCallback)(int status));

	// Suface Mesh Data Retrieval
	// documented in : http://reconstructme.net/wp-content/uploads/ReconstructMe/doc/group___surface_group.html
	// pass in pointer to float array and pointer to length
	// corrdiantes/indicies then reference arrays created by ReMe, size of array returned in length
	//reme_surface_get_points(reme_context_t c, reme_surface_t s, const float **coordinates, int *length)

	METAREME_API void getSurfacePoints(const float **coordinates, int *length);
	METAREME_API void getSurfaceNormals(const float **coordinates, int *length);
	METAREME_API void getSurfaceVertexColors(const float **coordinates, int *length);
	METAREME_API void getSurfaceTriangles(const size_t **indices, int *length);

	void runRenderLoop();
	void startSensor();
	void stopSensor();
	void setReconstructionOptions();
	bool getCurrentFrame();

	void logError(string error);
	void checkForErr(reme_error_t err, string msg);

	void remapToRGBA(reme_image_t remImage, uint8_t* rgbaImage, bool isColor);
	void sendColorImage();
	void sendVolumeImage();
#ifdef __cplusplus
}
#endif