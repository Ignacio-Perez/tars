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

#ifndef _SUBSTR_HPP_
#define _SUBSTR_HPP_

#include <string>

inline
unsigned substr(const std::string& str, char firstChar, char lastChar, unsigned& pos0) {
	pos0 = str.find_first_of(firstChar,pos0);
	if (pos0 == std::string::npos) {
		throw std::runtime_error("Invalid syntax: "+str);
	}
	++pos0;
	unsigned pos1 = str.find_first_of(lastChar,pos0);
	if (pos1 == std::string::npos || pos1==pos0) {
		throw std::runtime_error("Invalid syntax: "+str);
	}
	return pos1-pos0;
}
#endif