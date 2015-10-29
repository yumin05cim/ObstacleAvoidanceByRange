#include "ObstacleAvoidanceByRange.h"
#include "ObstacleAvoidanceByRangeImpl.h"

using namespace ssr;

ObstacleAvoidanceByRange_Impl::ObstacleAvoidanceByRange_Impl(){
}

ObstacleAvoidanceByRange_Impl::~ObstacleAvoidanceByRange_Impl(){
}

void ObstacleAvoidanceByRange_Impl::setRange(RTC::RangeData range){
	m_range_impl = range;
}

void ObstacleAvoidanceByRange_Impl::setFarthestObstacleDistance(double obstacleDistance){
	m_farthest_obstacleDistance_impl = obstacleDistance;
}


int ObstacleAvoidanceByRange_Impl::angleToIndex(double angle){

	int index = (angle - m_range_impl.config.minAngle) / m_range_impl.config.angularRes;

	return index;
}

bool ObstacleAvoidanceByRange_Impl::ObstacleDetection(double start_angle, double end_angle, bool flag){

	int smallest_index = -1;
	double shortest_distance = 1000000.0;
		
	for(int i = start_angle; i < end_angle; i++){
		if( m_range_impl.ranges[i] < 0.01 ){
			continue;
		}
		if( m_range_impl.ranges[i] < shortest_distance ){
			shortest_distance = m_range_impl.ranges[i];
			smallest_index = i;

			if( shortest_distance < m_farthest_obstacleDistance_impl ) flag = true;
			else flag = false;
		}
	}

	return flag;

}

