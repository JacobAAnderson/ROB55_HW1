

float median(std::vector<float> &v)
{
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}


sensor_msgs::LaserScan filter(const sensor_msgs::LaserScan &scan){

	sensor_msgs::LaserScan newScan = scan;

	int filterSize = 20;                                                        //	Size of filter
	int size = scan.ranges.size();                                             //	Size of the laser scan

	int a = filterSize/2;
	std::vector<float> ranges;
	std::vector<float> buff;

	for (int i = 0; i < size; i++){																					//	Go through the scanline

		// Sliding window filter
		for(int j = std::max(i-a, 0); j < std::min( i+a, size ); j++ ) buff.push_back( scan.ranges[j] );

		float m = median(buff); 																							//	Get median Value of the sliding window

		buff.clear();																													// 	Clear the buffer fo rthe next pass

		ranges.push_back(m);																									//	Keep Median value
		}

	newScan.ranges = ranges;																								//	Assign the filtered values to the laser scan


	return newScan;

}
