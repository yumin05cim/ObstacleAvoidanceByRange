
namespace ssr{

	class ObstacleAvoidanceByRange_Impl{

	private:
		RTC::RangeData m_range_impl;
		double m_farthest_obstacleDistance_impl;

	public:
		ObstacleAvoidanceByRange_Impl();
		~ObstacleAvoidanceByRange_Impl();

		void setRange(RTC::RangeData range);

		void ObstacleAvoidanceByRange_Impl::setFarthestObstacleDistance(double obstacleDistance);

		int angleToIndex(double angle);

		bool ObstacleDetection(double start_angle, double end_angle, bool flag);


	};

}