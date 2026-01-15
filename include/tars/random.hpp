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

#ifndef _RANDOM_HPP_
#define _RANDOM_HPP_

#include <algorithm>
#include <random>
#include <chrono>

class RandomNumberGenerator
{
public:
	RandomNumberGenerator(RandomNumberGenerator const&) = delete;
    void operator=(RandomNumberGenerator const&) = delete;
	~RandomNumberGenerator() {}
   
	static RandomNumberGenerator& getInstance()	{
    	static RandomNumberGenerator singleton;
    	return singleton;
	}
	#define RANDOM RandomNumberGenerator::getInstance()
	
	unsigned getSeed() const {
		return seed;
	}
		
	unsigned operator()(unsigned size) {
		std::uniform_int_distribution<unsigned> distribution(0,size-1);
		return distribution(gen);
	}
	
	double operator()() {
		return uniformDist(gen);
	}
	
	double operator()(double mean, double stddev) {
		std::normal_distribution<double> distribution(mean,stddev);
		return distribution(gen);
	}

	template<class T>
	void shuffle(T first, T last) {
		std::shuffle(first,last,gen);
	}

	std::mt19937& getGen() {
		return gen;
	}

private:
	RandomNumberGenerator(): seed(std::chrono::system_clock::now().time_since_epoch().count()), gen(seed), uniformDist(0.0,1.0) {} 
	unsigned seed;
	std::mt19937 gen;
	std::uniform_real_distribution<double> uniformDist;
};

#endif
