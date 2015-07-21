/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#include ".\gltexture.h"
#include <Utils/BMPIO.h>
#include <Utils/Image.h>
#include <Utils/Utils.h>
#include <iostream>


#define GL_TEXTURE_MAX_ANISOTROPY_EXT 0x84FE
#define GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT 0x84FF

/**
	This method checks the int value that is passed in as a parameter, and returns true if it is one of
	64, 128, 256 or 512, false otherwise
*/
bool checkTexSize(int x){
	return (x == 64 || x == 128 || x == 256 || x == 512 || x == 1024 || x ==2048);
}


/*
	The constructor takes as input the name of a .bmp file that contains the texture to be loaded.
	This constructor throws errors if the file cannot be found, or if it's height and width are not powers of 2
*/
GLTexture::GLTexture(char* fileName){
	
	std::vector<std::string> splited;
	splited= split(std::string(fileName), '.', splited);


	//load the image
	Image* im;

	if (splited.back() == "bmp"){
		BMPIO b(fileName);
		im = b.loadFromFile();

		if (splited[splited.size()-2] == "transpa"){
			int w = im->getWidth();
			int h = im->getHeight();
			byte *imageData = new byte[w* 4 * h];
			byte * imageDataPtr = imageData;
			byte * old_ptr = im->getDataPointer();
			for (int i = 0; i < h; ++i){
				for (int j = 0; j < w; ++j){
					*imageDataPtr++ = *old_ptr++;
					*imageDataPtr++ = *old_ptr++;
					*imageDataPtr++ = *old_ptr++;
					*imageDataPtr++ = 120;
				}
			}
			delete im;
			im = new Image(4, h, w, imageData);
		}
	}
	else if (splited.back() == "png"){
		byte *imageData = new byte[512*4* 512];
		byte * imageDataPtr = imageData;
		for (int i = 0; i < 512; ++i){
			for (int j = 0; j < 512; ++j){
				*imageDataPtr++ = 100;
				*imageDataPtr++ = 0;
				*imageDataPtr++ = 0;
				*imageDataPtr++ = 100;
			}
		}

		im= new Image(4, 512, 512, imageData);
	}
	else{
		throwError("the image ain't a bmp of a png");
	}
	texID = 0;

	//and generate the texture...
	glGenTextures(1, &texID);

	activate();

	if (!checkTexSize(im->getWidth()) || !checkTexSize(im->getHeight()) || (im->getNrBytes() != 3 && im->getNrBytes() != 4)){
		throwError("Wrong texture dimension or nr bits in file \'%s\'", fileName);
	}


	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); 
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); 
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 4);

//	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);

//	glTexImage2D(GL_TEXTURE_2D, 0, 3, im->getWidth(), im->getHeight(), 0, GL_RGB, GL_UNSIGNED_BYTE, im->getDataPointer());

	if (im->getNrBytes()==4){
		gluBuild2DMipmaps(GL_TEXTURE_2D, im->getNrBytes(), im->getWidth(), im->getHeight(), GL_RGBA, GL_UNSIGNED_BYTE, im->getDataPointer());

	}
	else {
		gluBuild2DMipmaps(GL_TEXTURE_2D, im->getNrBytes(), im->getWidth(), im->getHeight(), GL_RGB, GL_UNSIGNED_BYTE, im->getDataPointer());

	}
	
	delete im;
}
/*
	Destructor...
*/
GLTexture::~GLTexture(void){
	glDeleteTextures(1, &texID);
}




/*
	this method sets the current texture as the active
*/
void GLTexture::activate(){
	glBindTexture(GL_TEXTURE_2D, texID);
}


