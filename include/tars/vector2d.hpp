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

#ifndef _VECTOR2D_HPP_
#define _VECTOR2D_HPP_

#include <ostream>
#include <cmath>
#include "angle.hpp"

namespace utils
{
class Vector2d
{
public:
	Vector2d() : x(0),y(0) {}
	Vector2d(double x, double y) : x(x),y(y) {}
	virtual ~Vector2d() {}
	double operator()(int index) const {return index==0?x:y;}
	double operator[](int index) const {return index==0?x:y;}
	bool operator!=(const Vector2d& other) const {return x!=other.x || y!=other.y;}
	bool operator==(const Vector2d& other) const {return x==other.x && y==other.y;}
	bool operator<(const Vector2d& other) const {return x<other.x || (x==other.x && y<other.y);}
	double getX() const {return x;}
	double getY() const {return y;}

	Vector2d& set(double x, double y) {
		Vector2d::x = x;
		Vector2d::y = y;
		return *this;
	}

	Vector2d& setX(double x) {
		Vector2d::x = x;
		return *this;
	}
	
	Vector2d& setY(double y) {
		Vector2d::y = y;
		return *this;
	}	
	
	Vector2d& incX(double inc_x) {
		x += inc_x;
		return *this;
	}
	
	Vector2d& incY(double inc_y) {
		y += inc_y;
		return *this;
	}	

	Vector2d& inc(double inc_x, double inc_y) {
		x+=inc_x;
		y+=inc_y;
		return *this;
	}

	const Angle angle() const {
		return Angle::fromRadian(std::atan2(y,x));
	}

	Angle angleTo(const Vector2d& other) const {
		return other.angle() - angle();
	}

	double squaredNorm() const {
		return x*x + y*y;
	}

	double norm() const	{
		return std::sqrt(squaredNorm());
	}

	double dot(const Vector2d& other) const	{
		return x*other.x + y*other.y;
	}

	Vector2d& normalize() {
		double n = norm();
		if (n>0) {
			x /= n;
			y /= n;
		}		
		return *this;	
	}

	Vector2d normalized() const {
		Vector2d v(*this);
		v.normalize();
		return v;	
	}

	Vector2d& operator *=(double scalar) {
		x*=scalar;
		y*=scalar;
		return *this;
	}

	Vector2d operator *(double scalar) const {
		return Vector2d(x*scalar,y*scalar);
	}

	Vector2d& operator /=(double scalar) {
		x/=scalar;
		y/=scalar;
		return *this;
	}
	Vector2d operator /(double scalar) const {
		return Vector2d(x/scalar,y/scalar);
	}

	Vector2d leftNormalVector() const {
		return Vector2d(-y,x);
	}

	Vector2d rightNormalVector() const {
		return Vector2d(y,-x);
	}

	Vector2d& operator +=(const Vector2d& other) {
		set(x+other.x,y+other.y);
		return *this;
	}

	Vector2d operator +(const Vector2d& other) const {
		return Vector2d(x+other.x,y+other.y);
	}
	Vector2d& operator -=(const Vector2d& other) {
		set(x-other.x,y-other.y);
		return *this;
	}
	Vector2d operator -(const Vector2d& other) const {
		return Vector2d(x-other.x,y-other.y);
	}
	Vector2d operator -() const	{
		return Vector2d(-x,-y);
	}
	
	static const Vector2d& Zero() {
		static Vector2d zero;
		return zero;
	}

private:
	double x;
	double y;
};
}

inline
utils::Vector2d operator *(double scalar, const utils::Vector2d& v) {
	utils::Vector2d w(v);
	w*=scalar;
	return w;
}

namespace std
{
inline
ostream& operator<<(ostream& stream, const utils::Vector2d& v) {
	stream<<"("<<v.getX()<<","<<v.getY()<<")"; 
	return stream;
}
}
#endif