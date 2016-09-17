#ifndef PANO_INFO
#define PANO_INFO
#include <string>

#include <Eigen/Dense>

#include <cereal/archives/JSON.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>

struct gpsdata
{
	gpsdata()
	{

	}
	gpsdata(double x, double y, double z,
		double heading, double pitch, double roll)
	{
		_x = x;
		_y = y;
		_z = z;
		_heading = heading;
		_pitch = pitch;
		_roll = roll;
	}

	template <class Archive>
	void serialize(Archive & ar)
	{
		ar( cereal::make_nvp("x", _x),
			cereal::make_nvp("y", _y),
			cereal::make_nvp("z", _z),
			cereal::make_nvp("heading", _heading),
			cereal::make_nvp("pitch", _pitch),
			cereal::make_nvp("roll", _roll));
	}
	
	double _heading;
	double _pitch;
	double _roll;
	double _x;
	double _y;
	double _z;
};

struct frame_data
{
	frame_data():
	_frameName(""), _gpsData(gpsdata()), _R0(Eigen::Matrix<double, 3, 3>())
	{

	}
	template <class Archive>
	void serialize(Archive & ar)
	{
		ar(
			cereal::make_nvp("frameName", _frameName),
			cereal::make_nvp("gpsData", _gpsData),
			cereal::make_nvp("a1", _R0(0, 0)),
			cereal::make_nvp("a2", _R0(0, 1)),
			cereal::make_nvp("a3", _R0(0, 2)),
			cereal::make_nvp("b1", _R0(1, 0)),
			cereal::make_nvp("b2", _R0(1, 1)),
			cereal::make_nvp("b3", _R0(1, 2)),
			cereal::make_nvp("c1", _R0(2, 0)),
			cereal::make_nvp("c2", _R0(2, 1)),
			cereal::make_nvp("c3", _R0(2, 2))
			);
	}
	std::string _frameName;
	gpsdata _gpsData;
	Eigen::Matrix<double, 3, 3>  _R0;//frame -> pano
};

#endif // PANO_INFO