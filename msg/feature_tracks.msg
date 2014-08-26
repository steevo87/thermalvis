# This message defines meta information for a feature tracks structure. 
# It should preserve the header information for the latest image used 
# to generate it.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
Header header    # Header timestamp should be acquisition time of image
                 # Header frame_id should be optical frame of camera
                 # origin of frame should be optical center of camera
                 # +x should point to the right in the image
                 # +y should point down in the image
                 # +z should point into the plane of the image
                
#######################################################################
#                   	  Tracking info  	                          #
#######################################################################

# Source image topic from which tracks have been generated
string source

# Total number of projections included in this message
uint32 projection_count


# Camera indices (frame numbers) of each projection (in order)
uint32[] cameras
# Feature/track indices of each projection (in order)
int64[] indices
# X-location in image of the corresponding feature/track
float64[] projections_x
# Y-location in image of the corresponding feature/track
float64[] projections_y
