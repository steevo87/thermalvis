/*! \file	colormapping.hpp
 *  \brief	Declarations for colormapping.
*/

#ifndef THERMALVIS_COLORMAPPING_H
#define THERMALVIS_COLORMAPPING_H

#include "core/improc.hpp"

/// \brief		ID for custom mapping
namespace tv {
	#define MAP_LENGTH		1024

	#define MAX_INPUT_LENGTH	256

	#define GRAYSCALE		0

	#define CIECOMP			100
	#define CIECOMP_ALT_1	110
	#define CIECOMP_ALT_2	120
	#define CIECOMP_ALT_3	130

	#define CIELUV			140

	#define BLACKBODY		150

	#define HIGHLIGHTED		180

	#define RAINBOW			200
	#define RAINBOW_ALT_1	210
	#define RAINBOW_ALT_2	220
	#define RAINBOW_ALT_3	230
	#define RAINBOW_ALT_4	240

	#define IRON			300
	#define IRON_ALT_1		310
	#define IRON_ALT_2		320
	#define IRON_ALT_3		330

	#define BLUERED			400
	#define BLUERED_ALT_1	410
	#define BLUERED_ALT_2	420

	#define JET				500
	#define JET_ALT_1		510

	#define ICE				600
	#define ICE_ALT_1		610
	#define ICE_ALT_2		620
	#define ICE_ALT_3		630

	#define ICEIRON			700
	#define ICEIRON_ALT_1	710
	#define ICEIRON_ALT_2	720

	#define ICEFIRE			800
	#define ICEFIRE_ALT_1	810
	#define ICEFIRE_ALT_2	820
	#define ICEFIRE_ALT_3	830

	#define REPEATED		900
	#define REPEATED_ALT_1	910
	#define REPEATED_ALT_2	920
	#define REPEATED_ALT_3	930
	#define REPEATED_ALT_4	940
	#define REPEATED_ALT_5	950
	#define REPEATED_ALT_6	960

	#define MIN_PROP_THRESHOLD 0.002

	#define DEFAULT_LOWER_VISIBLE_FUSION_LIMIT 		0.2
	#define DEFAULT_UPPER_VISIBLE_FUSION_LIMIT 		0.8
}

/// \brief		For configuring and applying false-colormapping or modality-fusion schemes
class cScheme {
protected:
	
	/// \brief		Short array of the intensities corresponding to the color mappings (rx, gx, bx)
	double dx[MAX_INPUT_LENGTH];
	
	/// \brief		Short array of red-weightings of intensity mappings
	double rx[MAX_INPUT_LENGTH];
	/// \brief		Long array of red-weightings of intensity mappings
	double red[MAP_LENGTH];
	/// \brief		Short array of green-weightings of intensity mappings
	double gx[MAX_INPUT_LENGTH];
	/// \brief		Long array of green-weightings of intensity mappings
	double green[MAP_LENGTH];
	/// \brief		Short array of blue-weightings of intensity mappings
	double bx[MAX_INPUT_LENGTH];
	/// \brief		Long of blue-weightings of intensity mappings
	double blue[MAP_LENGTH];
	/// \brief		Number of elements in shortened colour arrays
	int		length;
	/// \brief		Code identifying which map is being used [see load_standard() function]
	int		code;

	/// \brief		Lookup table which shows what colour intensities correspond to what raw intensities
	unsigned char	lookupTable_1[256][3];
	unsigned short	lookupTable_2[65536][3];

public:
	/// \brief 		Constructor.
	cScheme();

	/// \brief 		Constructor.
	/// \param 		mapCode: 1 for safe, 0 for unsafe (with blacks and whites)
	cScheme(int mapCode, int mapParam = 0);

	/// \brief 		Destructor.
	~cScheme();

	/// \brief 		Load a standard colour map in as the scheme.
	/// \param 		mapCode: 1 for safe, 0 for unsafe (with blacks and whites)
	void load_standard(int mapCode, int mapParam = 0);

	/// \brief 		Creates a long map using the shortened map.
	void create_long_map();

	/// \brief		Recreates lookup table
	void setupLookupTable(unsigned int depth = 2);

	/// \brief 		Create and apply a new colour map as the scheme.
	/// \param 		r		Pointer to red weightings
	/// \param 		g		Pointer to green weightings
	/// \param 		b		Pointer to blue weightings
	/// \param 		len		Length of all three arrays
	// void customize(double* d, double* r, double* g, double* b, int len);

	/// \brief 		Create a false colour version of a monochromatic thermal image.
	/// \param 		thermIm		Monochrocv::Matic Thermal Image
	/// \param 		outputIm	Colour output Image
	void falsify_image(const cv::Mat& thermIm, cv::Mat& outputIm);

	/// \brief 		Create a false colour combination of a colour or monochrome visual image and a monochrome thermal image.
	/// \param 		thermIm		Monochromatic Thermal Image
	/// \param 		visualIm	Colour or monochromatic Visual Image
	/// \param 		outputIm	Colour output Image
	/// \param 		params		Pointer to double values dictating the following parameters:
	///							[0] : minimum lightness (default = 0.2)
	///							[1] : maximum lightness (default = 0.8)
	void fuse_image(cv::Mat& thermIm, cv::Mat& visualIm, cv::Mat& outputIm, double *params = NULL);
	
	/// \brief 		Combines RGB and thermal image using some other scheme
	void forge_image(cv::Mat& thermIm, cv::Mat& visualIm, cv::Mat& outputIm, double* params = NULL, double thresh = 0.05);

	/// \brief 		Returns code for current scheme.
	/// \param 		output		0 = rainbow, 1 = iron, 2 = jet, 3 = custom
	int current_scheme();

	/// \brief 		Resizes the given image, preserving the underlying data (to some extent).
	/// \param 		inputIm		Input image
	/// \param 		dim_i		Width
	/// \param 		dim_j		Height
	void image_resize(cv::Mat& inputIm, int dim_i, int dim_j);


};

#endif // THERMALVIS_COLORMAPPING_H
