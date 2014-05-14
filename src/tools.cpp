/*! \file	tools.cpp
 *  \brief	Definitions for generic tools not depending on libraries such as OpenCV, PCL and ROS.
*/

#include "tools.hpp"

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	double round(double d) {
		return floor(d + 0.5);
	}
#endif

bool comparator ( const mypair& l, const mypair& r) { 
	return l.first < r.first; 
}

void findLinearModel(double* x, double* y, int termsToConsider, double &m, double &c) {
	
	double mean_x = 0.0, mean_y = 0.0;
	
	if (termsToConsider == 1) {
		m = 0.0;
		c = y[0];
		
		return;
	}
	
	//printf("%s << termsToConsider = (%d)\n", __FUNCTION__, termsToConsider);
	
	for (int iii = 0; iii < termsToConsider; iii++) {
		mean_x += x[iii];
		mean_y += y[iii];
	}
	
	mean_x /= double(termsToConsider);
	mean_y /= double(termsToConsider);
	
	//printf("%s << means = (%f, %f)\n", __FUNCTION__, mean_x, mean_y);
	
	double s_x = 0.0, s_xy = 0.0; // s_y = 0.0, 
	
	for (int iii = 0; iii < termsToConsider; iii++) {
		s_x += pow(x[iii]-mean_x,2.0);
		//s_y += pow(y[iii]-mean_y,2.0);
		s_xy += (x[iii]-mean_x)*(y[iii]-mean_y);
	}
	
	//printf("%s << s's = (%f, %f)\n", __FUNCTION__, s_x, s_xy);
	
	m = s_xy / s_x;
	c = mean_y - m*mean_x;
	
	//printf("%s << eq = (%f, %f)\n", __FUNCTION__, m, c);
	
}





void convertUcharToBinary(unsigned char val, int* binaryArray) {
    for (int iii = 0; iii < 8; iii++) {
        if ((int) val >= (int) pow(2, 7-iii)) {
            binaryArray[iii] = 1;
            val -= (int) pow(2, 7-iii);
        } else {
            binaryArray[iii] = 0;
        }
    }
}

int countElementsInFolder(const char* folderName, vector<string>& elementNames, int elementType) {

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	printf("%s << ERROR! THIS FUNCTION HAS NOT BEEN IMPLEMENTED IN WINDOWS!\n", __FUNCTION__);
	return -1;
#else


    int elementCount = 0;
    DIR *dirp;
    struct dirent * entry;

    int typeCode = 0;

    if (elementType == 0) {
        // Wants to count files
        typeCode = DT_REG;
    } else if (elementType == 1) {
        // Wants to count folders
        typeCode = DT_DIR;
    }

    char *folder;

    folder = (char*) malloc(strlen(folderName) + 32);
    sprintf(folder, "%s", folderName);

    //printf("%s << folder = %s\n", __FUNCTION__, folder);

    dirp = opendir(folder);

    //printf("%s << folder opened.\n", __FUNCTION__);

    while ((entry = readdir(dirp)) != NULL) {
        //printf("%s << entry is: %s\n", __FUNCTION__, entry->d_name);
        if ((entry->d_type == typeCode) && (entry->d_name[0] != '.')) { // If the entry is a regular folder

           elementNames.push_back(string(entry->d_name));


           printf("%s << elementName[%d] = %s\n", __FUNCTION__, elementCount, elementNames.at(elementCount).c_str());

           elementCount++;

        }
    }

    closedir(dirp);

    return elementCount;
#endif
}



double timeElapsedMS(struct timeval& timer, bool reset) {

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	printf("%s << ERROR! THIS FUNCTION HAS NOT BEEN IMPLEMENTED IN WINDOWS!\n", __FUNCTION__);
	return -1.0;
#else
	struct timeval new_time;

	long seconds, useconds;

	gettimeofday(&new_time, NULL);

    seconds  = new_time.tv_sec  - timer.tv_sec;
    useconds = new_time.tv_usec - timer.tv_usec;

	double retVal = ((double) seconds) * 1000.0 + ((double) useconds) * 0.001;

	if (reset) {
		timer = new_time;
	}

    return retVal;
#endif
	


}



void randomSelection(vector<unsigned int>& src, vector<unsigned int>& dst, unsigned int max) {

	dst.clear();
	dst.insert(dst.end(), src.begin(), src.end());

	if (dst.size() <= max) {
		return;
	}

	while (dst.size() > max) {
		dst.erase(dst.begin() + (rand() % dst.size()));
	}

}

double asymmetricGaussianValue(double score, double mean, double loVar, double hiVar) {

	double zScore, sigma = 1.0, retVal;

	if (score == mean) {
		return 1.00;
	} else if (score > mean+3*hiVar) {
		return 0.00;
	} else if (score < mean-3*loVar) {
		return 0.00;
	} else if (score > mean) {
		sigma = abs(hiVar - mean);
	} else if (score < mean) {
		sigma = abs(loVar - mean);
	}

	zScore = (score - mean) / sigma;
	retVal = exp(-pow(zScore, 2.0)/2.0);

	return retVal;

}

void addUniqueToVector(vector<unsigned int>& dst, vector<unsigned int>& src) {

	for (unsigned int iii = 0; iii < src.size(); iii++) {

		bool alreadyAdded = false;

		for (unsigned int jjj = 0; jjj < dst.size(); jjj++) {
			if (dst.at(jjj) == src.at(iii)) {
				alreadyAdded = true;
			}


		}

		if (!alreadyAdded) {
			dst.push_back(src.at(iii));
		}

	}

}

double calcLinePerpDistance(double *line1, double *line2) {
    double retVal = 0.0;

    retVal = abs(line2[2] - line1[2]) / sqrt(pow(line1[0], 2) + pow(line1[1], 2));

    return retVal;
}



long long int factorial(int num)
{

    long long int result=1;
    for (int i=1; i<=num; ++i) {
        //result=result*=i;
        result *= 1;
	}
    return result;

}

void getNextCombo(vector<unsigned int>& currentIndices, int r, int n) {

    //bool maxed = false;
    bool valid = true;

    //printf("%s << Entered function.\n", __FUNCTION__);

    // If no indices tested, use default (0, 1, 2 etc)
    if (currentIndices.size() == 0)
    {
        for (int i = 0; i < r; i++)
        {
            currentIndices.push_back(i);
        }
    }
    else
    {

        // Going back each digit
        int i = 0;

        while (valid && (i < r))
        {
            //printf("%s << i = %d / %d\n", __FUNCTION__, i, r);
            // If current index is about to go over its maximum...
            if (int(currentIndices.at(currentIndices.size()-i-1)) > (n-2-i))
            {
                //printf("%s << digit #(%d) is valid; less than %d\n", __FUNCTION__, currentIndices.size()-i-1, n-2-i);
                i++;    // check out next index
            }
            else        // Otherwise, just increment it, fill in trailing digits and exit while loop
            {
                currentIndices.at(currentIndices.size()-i-1) = currentIndices.at(currentIndices.size()-i-1) + 1;
                for (int j = 0; j < i; j++)
                {
                    currentIndices.at(currentIndices.size()-i+j) = currentIndices.at(currentIndices.size()-i+j-1) + 1;
                }
                valid = false;
            }
        }


    }
}

void calcParameters(const vector<double>& v, double& mean, double& stdev) {
	
	double sum = 0.0;
	
	for (unsigned int iii = 0; iii < v.size(); iii++) {
		sum += v.at(iii);
	}
	
	mean = sum / v.size();

	double sq_sum = 0.0; 
	
	for (unsigned int iii = 0; iii < v.size(); iii++) {
		sq_sum += pow(v.at(iii)-mean,2.0);
	}
	
	stdev = std::sqrt(sq_sum / v.size());
	
}

double findEquivalentProbabilityScore(double* values, int quantity, double prob)
{

    //int i = 0;
    vector<double> listVector;
    double min = 1.00;
    int minIndex;

    // Push values into vector
    for (int j = 0; j < quantity; j++)
    {
        listVector.push_back(values[j]);
    }

    // Pop minimum values off vector until you've reached sufficient depth for the probability
	#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		while (listVector.size() >= (unsigned int)(max(int(prob*quantity), 1))) {
	#else
		while (listVector.size() >= (unsigned int)(std::max(int(prob*quantity), 1))) {
	#endif
    
        //printf("%s << listVector.size() = %d, prob*quantity = %d\n", __FUNCTION__, listVector.size(), int(prob*quantity));
        //cin.get();
        min = 9e99;

        for (unsigned int j = 0; j < listVector.size(); j++)
        {
            if (listVector.at(j) < min)
            {
                min = listVector.at(j);
                minIndex = j;
            }
        }

        listVector.erase(listVector.begin() + minIndex);

    }

    return min;

}

void convert_byte_to_binary_string(void* src, char* dst) {

	unsigned char* num;
	
	num = (unsigned char*) src;
	
	unsigned int factor = 128;
	
	for (unsigned int iii = 0; iii < 8; iii++) {
		if (*num > factor) {
			dst[iii] = '1';
		} else {
			dst[iii] = '0';
		}
		factor /= 2;
	}
	dst[8] = '\0';
}
