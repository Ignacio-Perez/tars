/*
MIT License

Copyright (c) 2021 Ignacio Pérez Hurtado de Mendoza

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _ANGLE_HPP_
#define _ANGLE_HPP_

#include <ostream>
#include <cmath>

namespace utils
{
class Angle
{
public:
	enum AngleRange{
		// [0, 2*pi) or [0°, 360°)
		PositiveOnlyRange,
		// (-pi, +pi] or (-180°, 180°]
		PositiveNegativeRange
	};

	Angle() : value(0) {}
	virtual ~Angle() {}

	static Angle fromRadian(double value) {
		return Angle(value);
	}

	static Angle fromDegree(double value) {
		return Angle(value / 180 * M_PI);
	}

	double toRadian(AngleRange range = PositiveNegativeRange) const {
		if(range == PositiveNegativeRange) {
			return value;
		} else {
			return (value>=0) ? value : (value+2*M_PI);
		}
	}

	double cos() const {
		return std::cos(value);
	}

	double sin() const {
		return std::sin(value);
	}

	double toDegree(AngleRange range = PositiveNegativeRange) const {
		double degreeValue = value * 180 / M_PI;
		if(range == PositiveNegativeRange) {
			return degreeValue;
		} else {
			return (degreeValue>=0) ? degreeValue : (degreeValue+360);
		}
	}

	void setRadian(double value) { 
		Angle::value = value; 
		normalize();
	}

	void setDegree(double value) {
		Angle::value = value / 180 * M_PI;
		normalize();
	}

	int sign() const {
		if(value == 0) {
			return 0;
		} else if(value > 0) {
			return 1;
		} else {
			return -1;
		}
	}
	
	Angle operator+(const Angle& other) const {
		return Angle(value + other.value);
	}
	
	Angle operator-(const Angle& other) const {
		return Angle(value - other.value);
	}
	
	Angle& operator+=(const Angle& other) {
		value += other.value;
		normalize();
		return *this;
	}
	
	Angle& operator-=(const Angle& other) {
		value -= other.value;
		normalize();
		return *this;
	}

	bool operator==(const Angle& other) const {
		return value == other.value;
	}

	bool operator!=(const Angle& other) const {
		return value != other.value;
	}

	bool operator<(const Angle& other) const {
		return value < other.value;
	}

	bool operator<=(const Angle& other) const {
		return value <= other.value;
	}

	bool operator>(const Angle& other) const {
		return value > other.value;
	}

	bool operator>=(const Angle& other) const {
		return value >= other.value;
	}

private:
	Angle(double value) {
		Angle::value = value;
		normalize();
	}

	void normalize() { 
		while(value <= -M_PI) 
			value += 2*M_PI; 
		while(value > M_PI) 
			value -= 2*M_PI; 
	}
	double value;
};
}

namespace std 
{
inline
ostream& operator<<(ostream& stream, const utils::Angle& alpha) {
	stream<<alpha.toRadian();
	return stream;
}
}

#endif