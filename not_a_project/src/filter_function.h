

sensor_msgs::LaserScan filter(const sensor_msgs::LaserScan &scan){

	sensor_msgs::LaserScan newScan = scan;

	std::cout << "Filter --> Got Scanline" << std::endl;

	return newScan;

}
