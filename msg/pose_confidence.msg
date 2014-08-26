# This message defines meta information for a pose confidence structure. 
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
#                   	  Confidence info  	                          #
#######################################################################

# Source image topic from which confidence has been generated
string source

# Total number of metrics included in this message
uint32 metric_count

# Confidence score of the corresponding metric
float64[] scores
