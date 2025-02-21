#ifndef _H_PRIMITIVE_TYPE_DECLS
#define _H_PRIMITIVE_TYPE_DECLS

#include <tuple>
#include <variant>

class Position {
  private:
	double latitude;
	double longitude;
	double altitude;
  public:
	Position() {};
  	// TODO: Provide constructor
  	Position(int value) {
		
		switch (value) {
			case 1: // Position(1)
				latitude = 53.94695239246725;
				longitude = -1.0324467581465242;
				altitude = 10;
				break;
			case 2: // Position(2)
				latitude = 53.94686967436447;
				longitude = -1.030863672537687;
				altitude = 10;
				break;
			case 3: // Position(3)
				latitude = 53.94727808774756;
				longitude = -1.0297376569126344;
				altitude = 10;
				break;
			default:
				return;
		};
	};

	double getLatitude() {
		return latitude;
	};

	double getLongitude() {
		return longitude;
	};

	double getAltitude() {
		return altitude;
	};
  	// TODO: Implement equality
  	friend bool operator ==(const Position& lhs, const Position& rhs) {
  		return std::tie(lhs.latitude,lhs.longitude,lhs.altitude) == std::tie(rhs.latitude,rhs.longitude,rhs.altitude);
  	};
};

#endif /* _H_PRIMITIVE_TYPE_DECLS */
